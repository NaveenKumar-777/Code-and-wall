-- =============================================================================
-- ADS127L11 SPI Master Controller  (FIXED v3)
-- =============================================================================
-- Device   : ADS127L11 400-kSPS 24-bit Delta-Sigma ADC (Texas Instruments)
-- FPGA Clk : 25 MHz — NO clock divider
--            SCLK = 12.5 MHz  (one SCLK cycle = 2 clk cycles = 80 ns)
--
-- SPI Mode : CPOL=0, CPHA=1  (Mode 1)
--   CPOL=0 → SCLK idles LOW
--   CPHA=1 → master drives SDI on rising  edge of SCLK
--            master samples SDO on falling edge of SCLK
--
-- KEY TIMING INSIGHT — registered outputs add 1-cycle pipeline delay:
--
--   All signal assignments (sclk_r, sdi_r, cs_n_r) are REGISTERED.
--   They appear on the output pin ONE clock cycle AFTER the assignment.
--
--   Cycle N   : FSM runs rising-edge branch → sets sclk_r='1', sdi_r=tx[31]
--   Cycle N+1 : SCLK pin goes HIGH, SDI pin shows tx[31]
--               ADC sees rising edge, latches SDI, begins driving SDO
--   Cycle N+1 : FSM runs falling-edge branch → sets sclk_r='0'
--                                            → rx_shift captures sdo HERE
--               sdo from ADC is valid: ADC had all of cycle N+1 to settle it
--               (SDO prop delay ~19 ns max << 40 ns clk period)
--   Cycle N+2 : SCLK pin goes LOW
--
-- CS TIMING (datasheet §6.8, IOVDD > 2 V):
--   td(CSSC)  : CS↓ to first SCLK↑ ≥ 10 ns
--               Achieved: CS_SETUP holds cs_n_r='0', sclk_r='0' for one full
--               40 ns cycle before ST_SHIFT raises SCLK → 40 ns margin. ✓
--
--   td(SCCS)  : last SCLK↓ to CS↑ ≥ 10 ns
--               FIX-A: CS_HOLD is now TWO cycles:
--                 Cycle 1 (ST_CS_HOLD)  : sclk_r='0', cs_n_r='0' (CS still LOW)
--                 Cycle 2 (ST_CS_HOLD_B): cs_n_r='1' (CS rises next cycle)
--               SCLK falls at end of last ST_SHIFT falling-edge cycle.
--               CS pin rises one cycle AFTER ST_CS_HOLD_B assigns cs_n_r='1',
--               giving a pin-level gap of 40 ns between SCLK↓ and CS↑. ✓
--
--   tw(CSH)   : CS high pulse ≥ 20 ns between transactions
--               For ADC reads: ST_DONE + ST_IDLE = at least 2 cycles = 80 ns. ✓
--               For RREG: ST_RREG_GAP = 5 cycles = 200 ns. ✓
--
-- FRAME FORMAT (32-bit, STATUS_EN=1 default):
--   TX: bits[31:24]=command, bits[23:0]=data (NOP=all zeros for ADC read)
--   RX: bits[31:24]=status,  bits[23:0]=24-bit ADC conversion result
--
-- CONTINUOUS STREAMING:
--   drdy_n falling edge latched by drdy_flag even when FSM is busy.
--   FSM services it immediately on return to IDLE — no samples missed.
--
-- REGISTER ACCESS:
--   Write: 1 frame  {0x80|addr, 0x00, 0x00, wdata}
--   Read : 2 frames — RREG cmd frame then NOP readback (CS-high gap between)
--
-- FIXES vs v2:
--   FIX-A : td(SCCS) violation — CS_HOLD split into ST_CS_HOLD + ST_CS_HOLD_B.
--            CS now stays LOW for one extra cycle after the last SCLK falling
--            edge, guaranteeing ≥ 40 ns between SCLK↓ and CS↑ on the pin.
--            Same fix applied to frame-2 hold (ST_CS_HOLD2 / ST_CS_HOLD2_B).
--   FIX-B : data_valid must NOT assert for OP_REG_WR. The write-frame echo
--            data would overwrite adc_data_r and trigger downstream consumers.
--            ST_DONE/OP_REG_WR now only latches status_byte; data_valid stays '0'.
--   FIX-C : busy is now a registered output (busy_r) driven inside the FSM
--            process, eliminating combinatorial glitches on state transitions.
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ads127l11_spi is
    generic (
        FRAME_BITS : integer := 32
    );
    port (
        clk             : in  std_logic;
        rst_n           : in  std_logic;

        -- SPI pins
        sclk            : out std_logic;
        cs_n            : out std_logic;
        sdi             : out std_logic;
        sdo             : in  std_logic;
        drdy_n          : in  std_logic;

        -- ADC conversion data
        adc_data        : out std_logic_vector(23 downto 0);
        status_byte     : out std_logic_vector(7  downto 0);
        data_valid      : out std_logic;
        sample_count    : out std_logic_vector(31 downto 0);

        -- Register access
        reg_addr        : in  std_logic_vector(4 downto 0);
        reg_wdata       : in  std_logic_vector(7 downto 0);
        reg_rdata       : out std_logic_vector(7 downto 0);
        reg_rdata_valid : out std_logic;
        reg_wr_en       : in  std_logic;
        reg_rd_en       : in  std_logic;

        busy            : out std_logic
    );
end entity ads127l11_spi;

architecture rtl of ads127l11_spi is

    -- FIX-A: added ST_CS_HOLD_B and ST_CS_HOLD2_B for the two-cycle CS deassertion
    type t_state is (
        ST_IDLE,
        ST_CS_SETUP,
        ST_SHIFT,
        ST_CS_HOLD,       -- cycle 1: SCLK low, CS still low
        ST_CS_HOLD_B,     -- cycle 2: CS deasserted (rises next clk cycle on pin)
        ST_DONE,
        ST_RREG_GAP,
        ST_CS_SETUP2,
        ST_SHIFT2,
        ST_CS_HOLD2,      -- cycle 1: SCLK low, CS still low  (frame 2)
        ST_CS_HOLD2_B,    -- cycle 2: CS deasserted           (frame 2)
        ST_DONE2
    );
    type t_op is (OP_ADC_READ, OP_REG_WR, OP_REG_RD);

    signal state        : t_state := ST_IDLE;
    signal current_op   : t_op    := OP_ADC_READ;

    -- DRDY synchroniser
    signal drdy_sync1   : std_logic := '1';
    signal drdy_sync2   : std_logic := '1';
    signal drdy_prev    : std_logic := '1';
    signal drdy_flag    : std_logic := '0';

    -- SPI output registers
    signal sclk_r       : std_logic := '0';
    signal cs_n_r       : std_logic := '1';
    signal sdi_r        : std_logic := '0';

    -- Shift registers
    signal tx_shift     : std_logic_vector(FRAME_BITS-1 downto 0) := (others => '0');
    signal rx_shift     : std_logic_vector(FRAME_BITS-1 downto 0) := (others => '0');

    -- 1-bit SCLK phase toggle
    -- '0' = next action is a RISING  edge (sclk_r → '1')
    -- '1' = next action is a FALLING edge (sclk_r → '0')
    signal sclk_phase   : std_logic := '0';

    -- Bit counter (FRAME_BITS down to 1, decremented on each falling edge)
    signal bit_cnt      : integer range 0 to FRAME_BITS := 0;

    -- Inter-frame gap counter
    -- 5 cycles @ 25 MHz = 200 ns CS-high gap between RREG frames.
    -- ADS127L11 requires min tw(CSH) = 20 ns. 200 ns is ample margin.
    signal gap_cnt      : integer range 0 to 5 := 0;

    -- Output registers
    signal adc_data_r        : std_logic_vector(23 downto 0) := (others => '0');
    signal status_byte_r     : std_logic_vector(7  downto 0) := (others => '0');
    signal data_valid_r      : std_logic := '0';
    signal sample_count_r    : unsigned(31 downto 0) := (others => '0');
    signal reg_rdata_r       : std_logic_vector(7  downto 0) := (others => '0');
    signal reg_rdata_valid_r : std_logic := '0';

    -- FIX-C: registered busy signal (glitch-free)
    signal busy_r            : std_logic := '0';

begin

    -- =========================================================================
    -- DRDY 2-FF synchroniser + falling-edge flag
    -- =========================================================================
    p_drdy : process(clk, rst_n)
    begin
        if rst_n = '0' then
            drdy_sync1 <= '1';
            drdy_sync2 <= '1';
            drdy_prev  <= '1';
            drdy_flag  <= '0';
        elsif rising_edge(clk) then
            drdy_sync1 <= drdy_n;
            drdy_sync2 <= drdy_sync1;
            drdy_prev  <= drdy_sync2;

            -- Set flag on falling edge of synchronised DRDY.
            -- Clear flag when FSM actually starts shifting an ADC read frame,
            -- so a DRDY pulse arriving during CS_SETUP is not lost.
            if drdy_sync2 = '0' and drdy_prev = '1' then
                drdy_flag <= '1';
            elsif state = ST_SHIFT and current_op = OP_ADC_READ then
                drdy_flag <= '0';
            end if;
        end if;
    end process p_drdy;

    -- =========================================================================
    -- Single-process FSM
    -- =========================================================================
    p_fsm : process(clk, rst_n)
        variable v_cmd : std_logic_vector(7 downto 0);
    begin
        if rst_n = '0' then
            state             <= ST_IDLE;
            current_op        <= OP_ADC_READ;
            sclk_r            <= '0';
            cs_n_r            <= '1';
            sdi_r             <= '0';
            tx_shift          <= (others => '0');
            rx_shift          <= (others => '0');
            sclk_phase        <= '0';
            bit_cnt           <= 0;
            gap_cnt           <= 0;
            adc_data_r        <= (others => '0');
            status_byte_r     <= (others => '0');
            data_valid_r      <= '0';
            sample_count_r    <= (others => '0');
            reg_rdata_r       <= (others => '0');
            reg_rdata_valid_r <= '0';
            busy_r            <= '0';   -- FIX-C

        elsif rising_edge(clk) then

            -- Default pulse signals to '0' each cycle
            data_valid_r      <= '0';
            reg_rdata_valid_r <= '0';

            case state is

                -- =============================================================
                -- IDLE: priority → reg_wr > reg_rd > ADC drdy_flag
                -- =============================================================
                when ST_IDLE =>
                    sclk_r     <= '0';
                    cs_n_r     <= '1';
                    sdi_r      <= '0';
                    sclk_phase <= '0';
                    busy_r     <= '0';  -- FIX-C: de-assert busy in IDLE

                    if reg_wr_en = '1' then
                        -- WREG command: bit7=1, bits[4:0]=address
                        v_cmd      := std_logic_vector(unsigned'("01000000") or
                                      ("000" & unsigned(reg_addr)));
                        tx_shift   <= v_cmd & x"00" & x"00" & reg_wdata;
                        current_op <= OP_REG_WR;
                        busy_r     <= '1';  -- FIX-C
                        state      <= ST_CS_SETUP;

                    elsif reg_rd_en = '1' then
                        -- RREG command: bits[5]=1, bits[4:0]=address
                        v_cmd      := std_logic_vector(unsigned'("00100000") or
                                      ("000" & unsigned(reg_addr)));
                        tx_shift   <= v_cmd & x"00" & x"00" & x"00";
                        current_op <= OP_REG_RD;
                        busy_r     <= '1';  -- FIX-C
                        state      <= ST_CS_SETUP;

                    elsif drdy_flag = '1' then
                        tx_shift   <= (others => '0');
                        current_op <= OP_ADC_READ;
                        busy_r     <= '1';  -- FIX-C
                        state      <= ST_CS_SETUP;
                    end if;

                -- =============================================================
                -- CS_SETUP: assert CS while SCLK stays LOW.
                --
                -- Registered pipeline means:
                --   This cycle  : cs_n_r='0' → CS pin falls NEXT cycle.
                --   This cycle  : sdi_r=tx[MSB] → SDI pin valid NEXT cycle.
                --   Next cycle  : ST_SHIFT begins; rising-edge branch sets
                --                 sclk_r='1' → SCLK pin rises the cycle AFTER.
                --
                -- Pin-level gap CS↓ → SCLK↑ = 2 cycles = 80 ns >> 10 ns (td_CSSC). ✓
                -- =============================================================
                when ST_CS_SETUP =>
                    cs_n_r     <= '0';
                    sclk_r     <= '0';
                    sdi_r      <= tx_shift(FRAME_BITS-1); -- pre-drive MSB (40 ns SDI setup)
                    sclk_phase <= '0';
                    bit_cnt    <= FRAME_BITS;
                    rx_shift   <= (others => '0');
                    state      <= ST_SHIFT;

                -- =============================================================
                -- SHIFT — CPHA=1 timing with registered output pipeline
                --
                -- sclk_phase='0' → RISING edge cycle:
                --   This cycle : sclk_r ← '1', sdi_r ← tx[MSB]
                --   Next cycle : SCLK pin HIGH, SDI pin shows tx[MSB]
                --   ADC action : latches SDI on rising SCLK, begins driving SDO
                --
                -- sclk_phase='1' → FALLING edge cycle:
                --   This cycle : SCLK has been HIGH on the pin for one full 40 ns.
                --                ADC SDO prop delay from SCLK↑ is ≤ 19 ns (datasheet).
                --                sdo is stable; capture directly into rx_shift.
                --                sclk_r ← '0'
                --   Next cycle : SCLK pin goes LOW.
                -- =============================================================
                when ST_SHIFT =>
                    sclk_phase <= not sclk_phase;

                    if sclk_phase = '0' then
                        -- ---- RISING edge ----
                        sclk_r <= '1';
                        sdi_r  <= tx_shift(FRAME_BITS-1);

                    else
                        -- ---- FALLING edge ----
                        sclk_r   <= '0';
                        rx_shift <= rx_shift(FRAME_BITS-2 downto 0) & sdo;
                        tx_shift <= tx_shift(FRAME_BITS-2 downto 0) & '0';
                        bit_cnt  <= bit_cnt - 1;

                        if bit_cnt = 1 then
                            state <= ST_CS_HOLD;
                        end if;
                    end if;

                -- =============================================================
                -- FIX-A: Two-cycle CS deassertion to meet td(SCCS) ≥ 10 ns.
                --
                -- ST_CS_HOLD   : sclk_r='0', cs_n_r='0' — CS pin remains LOW.
                --                Last SCLK falling edge occurred when the shift
                --                loop set sclk_r='0' with bit_cnt=1.
                --                SCLK pin fell at the START of this cycle.
                --
                -- ST_CS_HOLD_B : cs_n_r='1' — CS pin rises ONE cycle later.
                --
                -- Pin timeline:
                --   ST_SHIFT (bit1, fall) cycle   : sclk_r='0'
                --   ST_CS_HOLD cycle               : SCLK pin LOW; cs_n_r still '0'
                --   ST_CS_HOLD_B cycle             : SCLK pin LOW; cs_n_r='1' assigned
                --   Next cycle                     : CS pin goes HIGH
                --
                -- Gap on pin between SCLK↓ and CS↑ = 2 full cycles = 80 ns >> 10 ns. ✓
                -- =============================================================
                when ST_CS_HOLD =>
                    sclk_r <= '0';
                    cs_n_r <= '0';  -- CS stays LOW this cycle
                    sdi_r  <= '0';
                    state  <= ST_CS_HOLD_B;

                when ST_CS_HOLD_B =>
                    cs_n_r <= '1';  -- CS pin rises next cycle
                    sclk_r <= '0';
                    sdi_r  <= '0';
                    state  <= ST_DONE;

                -- =============================================================
                -- DONE
                -- =============================================================
                when ST_DONE =>
                    case current_op is

                        when OP_ADC_READ =>
                            status_byte_r  <= rx_shift(FRAME_BITS-1 downto FRAME_BITS-8);
                            adc_data_r     <= rx_shift(23 downto 0);
                            data_valid_r   <= '1';   -- pulse for one cycle
                            sample_count_r <= sample_count_r + 1;
                            state          <= ST_IDLE;

                        when OP_REG_WR =>
                            -- FIX-B: Do NOT assert data_valid for a register write.
                            -- The echo frame is not an ADC conversion result.
                            -- Only latch the status byte for diagnostic use.
                            status_byte_r <= rx_shift(FRAME_BITS-1 downto FRAME_BITS-8);
                            -- data_valid_r intentionally left '0' (default above)
                            state         <= ST_IDLE;

                        when OP_REG_RD =>
                            tx_shift <= (others => '0');
                            gap_cnt  <= 5;  -- 5 cycles × 40 ns = 200 ns CS-high gap
                            state    <= ST_RREG_GAP;

                    end case;

                -- =============================================================
                -- RREG_GAP: CS-high gap between the two register-read frames.
                -- 5 cycles = 200 ns >> tw(CSH) = 20 ns. ✓
                -- =============================================================
                when ST_RREG_GAP =>
                    cs_n_r <= '1';
                    sclk_r <= '0';
                    if gap_cnt = 0 then
                        state <= ST_CS_SETUP2;
                    else
                        gap_cnt <= gap_cnt - 1;
                    end if;

                -- =============================================================
                -- Frame 2 (RREG NOP readback) — same corrected timing as frame 1
                -- =============================================================
                when ST_CS_SETUP2 =>
                    cs_n_r     <= '0';
                    sclk_r     <= '0';
                    sdi_r      <= tx_shift(FRAME_BITS-1);
                    sclk_phase <= '0';
                    bit_cnt    <= FRAME_BITS;
                    rx_shift   <= (others => '0');
                    state      <= ST_SHIFT2;

                when ST_SHIFT2 =>
                    sclk_phase <= not sclk_phase;

                    if sclk_phase = '0' then
                        sclk_r <= '1';
                        sdi_r  <= tx_shift(FRAME_BITS-1);
                    else
                        sclk_r   <= '0';
                        rx_shift <= rx_shift(FRAME_BITS-2 downto 0) & sdo;
                        tx_shift <= tx_shift(FRAME_BITS-2 downto 0) & '0';
                        bit_cnt  <= bit_cnt - 1;

                        if bit_cnt = 1 then
                            state <= ST_CS_HOLD2;
                        end if;
                    end if;

                -- FIX-A (frame 2): same two-cycle CS deassertion
                when ST_CS_HOLD2 =>
                    sclk_r <= '0';
                    cs_n_r <= '0';  -- CS stays LOW this cycle
                    sdi_r  <= '0';
                    state  <= ST_CS_HOLD2_B;

                when ST_CS_HOLD2_B =>
                    cs_n_r <= '1';  -- CS pin rises next cycle
                    sclk_r <= '0';
                    sdi_r  <= '0';
                    state  <= ST_DONE2;

                when ST_DONE2 =>
                    reg_rdata_r       <= rx_shift(7 downto 0);
                    reg_rdata_valid_r <= '1';
                    state             <= ST_IDLE;

                when others =>
                    state <= ST_IDLE;

            end case;
        end if;
    end process p_fsm;

    -- =========================================================================
    -- Output assignments
    -- =========================================================================
    sclk            <= sclk_r;
    cs_n            <= cs_n_r;
    sdi             <= sdi_r;
    adc_data        <= adc_data_r;
    status_byte     <= status_byte_r;
    data_valid      <= data_valid_r;
    sample_count    <= std_logic_vector(sample_count_r);
    reg_rdata       <= reg_rdata_r;
    reg_rdata_valid <= reg_rdata_valid_r;
    busy            <= busy_r;   -- FIX-C: registered, glitch-free

end architecture rtl;
