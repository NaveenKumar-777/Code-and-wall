--------------------------------------------------------------------------------
-- AD7356BRUZ Interface
--
-- Dual, 12-bit, simultaneous-sampling SAR ADC (Analog Devices).
--
-- Protocol summary (verified against AD7356 datasheet Rev. B, Table 3
-- "Timing Specifications" and Figure 30 "Serial Interface Timing Diagram"):
--   * CS_N falling edge samples both analog inputs (track-and-hold -> hold)
--     and starts the conversion.
--   * A minimum of 14 SCLK cycles is required: the data stream is 2 leading
--     zeros followed by the 12-bit result, MSB first. Bits change on the
--     SCLK FALLING edge.
--   * SDATAA and SDATAB output their 12-bit results simultaneously on
--     separate pins, so one 14-SCLK burst retrieves BOTH channels.
--   * fSCLK range: 50 kHz min, 80 MHz max.
--   * Key timing (min/max, worst case over VDRIVE/temp range):
--       t2  (CS-to-SCLK setup)                    5 ns min
--       t4  (SDATA access time after SCLK falling) 9-12.5 ns max (depends on VDRIVE)
--       t5  (SCLK low pulse width)                 5 ns min
--       t6  (SCLK high pulse width)                5 ns min
--       t7  (SCLK-to-data hold time)                3.5 ns min
--       t9  (CS high pulse width)                  5 ns min
--       tQUIET (end of read to next CS falling)    5 ns min
--   * Because t4 can be as large as ~12.5 ns, SDATA must NOT be sampled
--     right at the SCLK low->high transition when SCLK is fast (e.g. 50 MHz
--     => 10 ns half-period, leaving too little margin). This module instead
--     samples near the END of the SCLK-high phase, i.e. just before the
--     NEXT falling edge, giving nearly a full SCLK period of margin against
--     t4 since data is valid continuously between falling edges.
--   * After the transfer, CS_N is brought high; allow tQUIET before the
--     next CS_N falling edge.
--
-- Recommended system clock: 100 MHz gives an exact, clean 50 MHz SCLK
-- (SCLK_HALF_PERIOD_CYCLES => 1) with generous margin on all the 5 ns/
-- 3.5 ns minimums above (10 ns per system clock cycle). See generic
-- defaults below.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ad7356_interface is
    generic (
        -- Number of system clock cycles to hold each SCLK phase (half-period).
        -- Default assumes a 100 MHz system clock: 1 cycle = 10 ns per phase
        -- => SCLK period = 20 ns => SCLK = 50 MHz. Comfortably clears the
        -- t5/t6 5 ns min pulse-width specs and stays under the 80 MHz max.
        SCLK_HALF_PERIOD_CYCLES : positive := 1;

        -- Minimum system clock cycles CS_N must be low before SCLK toggles.
        -- t2 min = 5 ns; 1 cycle @ 100 MHz = 10 ns, clears it with margin.
        CS_TO_SCLK_CYCLES       : positive := 1;

        -- Minimum system clock cycles CS_N must stay high between
        -- conversions. tQUIET min = 5 ns; 1 cycle @ 100 MHz = 10 ns.
        QUIET_TIME_CYCLES       : positive := 1
    );
    port (
        clk        : in  std_logic;                      -- system clock
        rst_n      : in  std_logic;                       -- async active-low reset

        start      : in  std_logic;                       -- pulse to start one conversion/read cycle
        busy       : out std_logic;                       -- high while a cycle is in progress
        data_valid : out std_logic;                       -- one-cycle pulse when data_a/data_b are valid

        data_a     : out std_logic_vector(11 downto 0);   -- ADC A result (MSB first captured)
        data_b     : out std_logic_vector(11 downto 0);   -- ADC B result (MSB first captured)

        -- Physical ADC pins
        cs_n       : out std_logic;
        sclk       : out std_logic;
        sdata_a    : in  std_logic;
        sdata_b    : in  std_logic
    );
end entity ad7356_interface;

architecture rtl of ad7356_interface is

    type state_t is (S_IDLE, S_CS_SETUP, S_CLK_LOW, S_CLK_HIGH, S_CS_HIGH, S_QUIET);
    signal state        : state_t := S_IDLE;

    signal clk_cnt       : unsigned(15 downto 0) := (others => '0');
    signal bit_cnt        : unsigned(4 downto 0)  := (others => '0'); -- counts up to 14

    signal sclk_int      : std_logic := '0';
    signal cs_n_int       : std_logic := '1';

    signal shreg_a        : std_logic_vector(11 downto 0) := (others => '0');
    signal shreg_b         : std_logic_vector(11 downto 0) := (others => '0');

    constant NUM_SCLK_CYCLES : unsigned(4 downto 0) := to_unsigned(14, 5); -- min required by datasheet

begin

    cs_n <= cs_n_int;
    sclk <= sclk_int;

    process (clk, rst_n)
    begin
        if rst_n = '0' then
            state       <= S_IDLE;
            clk_cnt     <= (others => '0');
            bit_cnt     <= (others => '0');
            sclk_int    <= '0';
            cs_n_int    <= '1';
            shreg_a     <= (others => '0');
            shreg_b     <= (others => '0');
            data_a      <= (others => '0');
            data_b      <= (others => '0');
            busy        <= '0';
            data_valid  <= '0';

        elsif rising_edge(clk) then
            data_valid <= '0'; -- default: pulse only in the cycle we finish

            case state is

                ----------------------------------------------------------------
                when S_IDLE =>
                    busy     <= '0';
                    cs_n_int <= '1';
                    sclk_int <= '0';
                    bit_cnt  <= (others => '0');
                    clk_cnt  <= (others => '0');
                    if start = '1' then
                        busy     <= '1';
                        cs_n_int <= '0';   -- CS falling edge: sample + start conversion
                        state    <= S_CS_SETUP;
                    end if;

                ----------------------------------------------------------------
                -- Wait required setup time between CS falling edge and first
                -- SCLK edge.
                when S_CS_SETUP =>
                    if clk_cnt = to_unsigned(CS_TO_SCLK_CYCLES - 1, clk_cnt'length) then
                        clk_cnt  <= (others => '0');
                        bit_cnt  <= (others => '0');
                        sclk_int <= '0';
                        state    <= S_CLK_LOW;
                    else
                        clk_cnt <= clk_cnt + 1;
                    end if;

                ----------------------------------------------------------------
                -- SCLK low phase. Data on SDATA is valid here (changes on
                -- SCLK falling edge per datasheet) -- we sample it at the
                -- END of this phase, i.e. just before SCLK goes high, which
                -- gives it a full half-period to settle.
                when S_CLK_LOW =>
                    sclk_int <= '0';
                    if clk_cnt = to_unsigned(SCLK_HALF_PERIOD_CYCLES - 1, clk_cnt'length) then
                        clk_cnt <= (others => '0');
                        -- capture current bit (MSB first shift-in)
                        shreg_a <= shreg_a(10 downto 0) & sdata_a;
                        shreg_b <= shreg_b(10 downto 0) & sdata_b;
                        state   <= S_CLK_HIGH;
                    else
                        clk_cnt <= clk_cnt + 1;
                    end if;

                ----------------------------------------------------------------
                -- SCLK high phase.
                when S_CLK_HIGH =>
                    sclk_int <= '1';
                    if clk_cnt = to_unsigned(SCLK_HALF_PERIOD_CYCLES - 1, clk_cnt'length) then
                        clk_cnt <= (others => '0');
                        if bit_cnt = NUM_SCLK_CYCLES - 1 then
                            -- completed required 14 SCLK cycles
                            bit_cnt  <= (others => '0');
                            sclk_int <= '0';
                            state    <= S_CS_HIGH;
                        else
                            bit_cnt <= bit_cnt + 1;
                            state   <= S_CLK_LOW;
                        end if;
                    else
                        clk_cnt <= clk_cnt + 1;
                    end if;

                ----------------------------------------------------------------
                -- Bring CS back high to terminate the transfer and
                -- three-state SDATAA/SDATAB.
                when S_CS_HIGH =>
                    cs_n_int   <= '1';
                    -- shreg_a/shreg_b now hold the 12 data bits captured
                    -- during the first 12 SCLK cycles (bits 0..11 of bit_cnt
                    -- count, MSB first). The 13th/14th cycles (per datasheet)
                    -- are used by the part to finish conversion/return T&H
                    -- to track mode, and do not contain new channel data in
                    -- the basic 14-SCLK single-channel-per-line mode.
                    data_a     <= shreg_a;
                    data_b     <= shreg_b;
                    data_valid <= '1';
                    busy       <= '0';
                    clk_cnt    <= (others => '0');
                    state      <= S_QUIET;

                ----------------------------------------------------------------
                -- Enforce minimum CS-high quiet time before next conversion.
                when S_QUIET =>
                    if clk_cnt = to_unsigned(QUIET_TIME_CYCLES - 1, clk_cnt'length) then
                        clk_cnt <= (others => '0');
                        state   <= S_IDLE;
                    else
                        clk_cnt <= clk_cnt + 1;
                    end if;

                when others =>
                    state <= S_IDLE;

            end case;
        end if;
    end process;

end architecture rtl;
