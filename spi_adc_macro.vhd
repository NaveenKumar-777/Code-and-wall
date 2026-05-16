-------------------------------------------------------------------------------
-- SPI ADC Macro
--
-- SCLK is driven combinatorially from state AND last_bit flag so there is
-- zero latency between the final bit and SCLK stopping.
--
-- Bug fix: previously sclk_run depended only on FSM state (registered).
-- After the last fall_sclk edge the FSM state hadn't changed yet, so
-- sclk_run stayed '1' for one more system-clock cycle, producing an extra
-- (17th / 25th) rising edge.  Fix: sclk_run is now also gated by last_bit,
-- a combinatorial signal that goes '1' when the FSM is about to exit the
-- shift state (bit_cnt has reached its terminal count).  This cuts sclk_run
-- immediately on the same cycle the last falling edge is detected, preventing
-- the stray rising edge.
--
-- Register Write  (reg_data[15]='1'):
--   CS low -> 2 clk delay -> 16 SCLKs, SDI driven on rising edge -> CS high
--
-- Register Read   (reg_data[15]='0'):
--   Frame1: CS low -> 2 clk delay -> 16 SCLKs send command -> CS high 4 clks
--   Frame2: CS low -> 2 clk delay -> 16 SCLKs, sample SDO on falling -> CS high
--   reg_data_out latched on CS high. adc_data_en NOT asserted.
--
-- ADC Read  (adc_enable=1, drdy=0):
--   CS low -> 24 SCLKs, sample SDO on falling -> CS high
--   adc_data_out and adc_data_en updated on ADC_DONE.
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity spi_adc_macro is
    port (
        clk           : in  std_logic;
        reset         : in  std_logic;

        -- SPI interface
        sdo           : in  std_logic;
        sdi           : out std_logic;
        cs            : out std_logic;
        sclk          : out std_logic;
        drdy          : in  std_logic;

        -- Control
        adc_enable    : in  std_logic;
        reg_wr_rd_en  : in  std_logic;
        -- reg_data[15]: '1'=write, '0'=read
        -- reg_data[14:0]: command bits, sent MSB first on SDI
        reg_data      : in  std_logic_vector(15 downto 0);

        -- Outputs
        adc_data_out  : out std_logic_vector(23 downto 0);
        adc_data_en   : out std_logic;
        reg_data_out  : out std_logic_vector(15 downto 0)
    );
end entity spi_adc_macro;

architecture rtl of spi_adc_macro is

    ---------------------------------------------------------------------------
    -- FSM states
    ---------------------------------------------------------------------------
    type t_state is (
        IDLE,
        WR_CS_LOW, WR_DELAY, WR_SHIFT, WR_CS_HIGH,
        RD_CS_LOW,  RD_DELAY,  RD_SHIFT_CMD, RD_CS_GAP,
        RD_CS_LOW2, RD_DELAY2, RD_SHIFT_DATA, RD_CS_HIGH,
        ADC_CS_LOW, ADC_SHIFT, ADC_CS_HIGH, ADC_DONE
    );
    signal state : t_state := IDLE;

    ---------------------------------------------------------------------------
    -- SCLK: free-running toggle, gated combinatorially by sclk_run
    -- sclk_run is a pure combinatorial decode of state — no register latency
    ---------------------------------------------------------------------------
    signal sclk_int  : std_logic := '0';
    signal sclk_run  : std_logic;          -- combinatorial gate
    signal last_bit  : std_logic;          -- combinatorial: final bit reached, kill SCLK
    signal sclk_prev : std_logic := '0';
    signal rise_sclk : std_logic;
    signal fall_sclk : std_logic;

    ---------------------------------------------------------------------------
    -- Datapath
    ---------------------------------------------------------------------------
    signal cs_int       : std_logic := '1';
    signal sdi_int      : std_logic := '0';
    signal shift_reg    : std_logic_vector(15 downto 0) := (others => '0');
    signal rx_reg       : std_logic_vector(23 downto 0) := (others => '0');
    signal bit_cnt      : integer range 0 to 31 := 0;
    signal delay_cnt    : integer range 0 to 3  := 0;
    signal reg_data_lat : std_logic_vector(15 downto 0) := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- last_bit: combinatorial flag that goes '1' on the cycle where the FSM
    -- detects the terminal fall_sclk edge.  Gating sclk_run with (not last_bit)
    -- kills the clock immediately, preventing one spurious extra rising edge.
    ---------------------------------------------------------------------------
    last_bit <= '1' when (
                    (state = RD_SHIFT_CMD  and rise_sclk = '1' and bit_cnt = 15) or
                    (state = RD_SHIFT_DATA and fall_sclk = '1' and bit_cnt = 15) or
                    (state = ADC_SHIFT     and fall_sclk = '1' and bit_cnt = 23)
                ) else '0';

    ---------------------------------------------------------------------------
    -- SCLK is enabled when FSM is in a shift state AND last_bit not reached.
    -- WR_SHIFT exits on rise_sclk (last rising edge) so no last_bit needed
    -- there — the state exits on rise, and sclk goes low naturally.
    ---------------------------------------------------------------------------
    sclk_run <= '1' when (state = WR_SHIFT     or
                          state = RD_SHIFT_CMD  or
                          state = RD_SHIFT_DATA or
                          state = ADC_SHIFT)
                         and last_bit = '0'
                else '0';

    p_sclk : process(clk, reset)
    begin
        if reset = '1' then
            sclk_int <= '0';
        elsif rising_edge(clk) then
            if sclk_run = '1' then
                sclk_int <= not sclk_int;
            else
                sclk_int <= '0';
            end if;
        end if;
    end process;

    p_edge : process(clk, reset)
    begin
        if reset = '1' then
            sclk_prev <= '0';
        elsif rising_edge(clk) then
            sclk_prev <= sclk_int;
        end if;
    end process;

    rise_sclk <= '1' when (sclk_prev = '0' and sclk_int = '1') else '0';
    fall_sclk <= '1' when (sclk_prev = '1' and sclk_int = '0') else '0';

    ---------------------------------------------------------------------------
    -- FSM
    ---------------------------------------------------------------------------
    p_fsm : process(clk, reset)
    begin
        if reset = '1' then
            state        <= IDLE;
            cs_int       <= '1';
            sdi_int      <= '0';
            bit_cnt      <= 0;
            delay_cnt    <= 0;
            shift_reg    <= (others => '0');
            rx_reg       <= (others => '0');
            adc_data_out <= (others => '0');
            adc_data_en  <= '0';
            reg_data_out <= (others => '0');
            reg_data_lat <= (others => '0');

        elsif rising_edge(clk) then
            adc_data_en <= '0';

            case state is

                ---------------------------------------------------------------
                when IDLE =>
                    cs_int    <= '1';
                    sdi_int   <= '0';
                    bit_cnt   <= 0;
                    delay_cnt <= 0;
                    if reg_wr_rd_en = '1' then
                        reg_data_lat <= reg_data;
                        if reg_data(15) = '1' then
                            state <= WR_CS_LOW;
                        else
                            state <= RD_CS_LOW;
                        end if;
                    elsif adc_enable = '1' and drdy = '0' then
                        state <= ADC_CS_LOW;
                    end if;

                ---------------------------------------------------------------
                -- WRITE
                ---------------------------------------------------------------
                when WR_CS_LOW =>
                    cs_int    <= '0';
                    delay_cnt <= 0;
                    state     <= WR_DELAY;

                when WR_DELAY =>
                    if delay_cnt = 1 then
                        shift_reg <= reg_data_lat;
                        bit_cnt   <= 0;
                        state     <= WR_SHIFT;
                    else
                        delay_cnt <= delay_cnt + 1;
                    end if;

                -- sclk_run='1' here (combinatorial), SCLK toggles this cycle
                when WR_SHIFT =>
                    if rise_sclk = '1' then
                        sdi_int   <= shift_reg(15);
                        shift_reg <= shift_reg(14 downto 0) & '0';
                        if bit_cnt = 15 then
                            state <= WR_CS_HIGH;    -- sclk_run goes '0' immediately
                        else
                            bit_cnt <= bit_cnt + 1;
                        end if;
                    end if;

                when WR_CS_HIGH =>
                    cs_int  <= '1';
                    sdi_int <= '0';
                    state   <= IDLE;

                ---------------------------------------------------------------
                -- READ FRAME 1
                ---------------------------------------------------------------
                when RD_CS_LOW =>
                    cs_int    <= '0';
                    delay_cnt <= 0;
                    state     <= RD_DELAY;

                when RD_DELAY =>
                    if delay_cnt = 1 then
                        shift_reg <= reg_data_lat;
                        bit_cnt   <= 0;
                        state     <= RD_SHIFT_CMD;
                    else
                        delay_cnt <= delay_cnt + 1;
                    end if;

                when RD_SHIFT_CMD =>
                    if rise_sclk = '1' then
                        sdi_int   <= shift_reg(15);
                        shift_reg <= shift_reg(14 downto 0) & '0';
                        if bit_cnt = 15 then
                            delay_cnt <= 0;
                            state     <= RD_CS_GAP;
                        else
                            bit_cnt <= bit_cnt + 1;
                        end if;
                    end if;

                when RD_CS_GAP =>               -- CS high 4 clocks
                    cs_int  <= '1';
                    sdi_int <= '0';
                    if delay_cnt = 3 then
                        delay_cnt <= 0;
                        state     <= RD_CS_LOW2;
                    else
                        delay_cnt <= delay_cnt + 1;
                    end if;

                ---------------------------------------------------------------
                -- READ FRAME 2
                ---------------------------------------------------------------
                when RD_CS_LOW2 =>
                    cs_int    <= '0';
                    delay_cnt <= 0;
                    state     <= RD_DELAY2;

                when RD_DELAY2 =>               -- 2-clock delay (counts 0,1)
                    rx_reg  <= (others => '0');
                    bit_cnt <= 0;
                    if delay_cnt = 1 then
                        state <= RD_SHIFT_DATA;
                    else
                        delay_cnt <= delay_cnt + 1;
                    end if;

                -- sclk_run='1' here (combinatorial)
                when RD_SHIFT_DATA =>
                    if fall_sclk = '1' then
                        rx_reg(15 downto 0) <= rx_reg(14 downto 0) & sdo;
                        if bit_cnt = 15 then
                            state <= RD_CS_HIGH;
                        else
                            bit_cnt <= bit_cnt + 1;
                        end if;
                    end if;

                when RD_CS_HIGH =>
                    cs_int       <= '1';
                    reg_data_out <= rx_reg(15 downto 0);
                    state        <= IDLE;

                ---------------------------------------------------------------
                -- ADC READ
                ---------------------------------------------------------------
                when ADC_CS_LOW =>
                    cs_int  <= '0';
                    bit_cnt <= 0;
                    rx_reg  <= (others => '0');
                    sdi_int <= '0';
                    state   <= ADC_SHIFT;

                -- sclk_run='1' here (combinatorial)
                when ADC_SHIFT =>
                    if fall_sclk = '1' then
                        rx_reg <= rx_reg(22 downto 0) & sdo;
                        if bit_cnt = 23 then
                            state <= ADC_CS_HIGH;
                        else
                            bit_cnt <= bit_cnt + 1;
                        end if;
                    end if;

                when ADC_CS_HIGH =>
                    cs_int <= '1';
                    state  <= ADC_DONE;

                when ADC_DONE =>
                    adc_data_out <= rx_reg;
                    adc_data_en  <= '1';
                    state        <= IDLE;

                when others =>
                    state <= IDLE;

            end case;
        end if;
    end process;

    cs   <= cs_int;
    sdi  <= sdi_int;
    sclk <= sclk_int;

end architecture rtl;






--add_force {/spi_adc_macro/clk} -radix hex {1 0ns} {0 5000ps} -repeat_every 10000ps
--add_force {/spi_adc_macro/reset} -radix hex {1 0ns}
--run 100 ns
--run 100 ns
--add_force {/spi_adc_macro/reset} -radix hex {0 0ns}
--run 100 ns
--run 100 ns
--run 100 ns
--add_force {/spi_adc_macro/drdy} -radix hex {1 0ns}
--add_force {/spi_adc_macro/adc_enable} -radix hex {0 0ns}
--add_force {/spi_adc_macro/reg_wr_rd_en} -radix hex {0 0ns}
--add_force {/spi_adc_macro/drdy} -radix hex {1 0ns}
--add_force {/spi_adc_macro/adc_enable} -radix hex {0 0ns}
--add_force {/spi_adc_macro/reg_wr_rd_en} -radix hex {0 0ns}
--add_force {/spi_adc_macro/reg_data} -radix hex {4100 0ns}
--run 100 ns
--add_force {/spi_adc_macro/sdo} -radix hex {0 0ns}
--run 100 ns
--add_force {/spi_adc_macro/sdo} -radix hex {1 0ns}
--run 100 ns
--add_force {/spi_adc_macro/reg_wr_rd_en} -radix hex {1 0ns}
--run 10 ns
--add_force {/spi_adc_macro/reg_wr_rd_en} -radix hex {0 0ns}
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--add_force {/spi_adc_macro/reg_data} -radix hex {8100 0ns}
--run 10 ns
--add_force {/spi_adc_macro/reg_wr_rd_en} -radix hex {1 0ns}
--run 10 ns
--add_force {/spi_adc_macro/reg_wr_rd_en} -radix hex {0 0ns}
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--add_force {/spi_adc_macro/adc_enable} -radix hex {1 0ns}
--run 10 ns
--run 10 ns
--run 10 ns
--run 10 ns
--run 10 ns
--add_force {/spi_adc_macro/drdy} -radix hex {0 0ns}
--run 10 ns
--add_force {/spi_adc_macro/drdy} -radix hex {1 0ns}
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns
--run 100 ns

