--------------------------------------------------------------------------------
-- rr_channel_scheduler.vhd
--
-- 16-channel round-robin scheduler for ADS127L11 SPI ADC macros.
--
-- Behavior:
--   1. When adc_channel_reg_wr pulses, load the lowest-numbered enabled
--      channel from adc_channel_reg and issue a one-clock start pulse on
--      channel_done_en for that channel.
--   2. Wait for channel_done(current_ch) to pulse (SPI macro finished its
--      conversion/FIFO write for the active channel). Stale/foreign
--      channel_done pulses (from a channel that is not the active one) are
--      ignored.
--   3. On channel_done(current_ch), advance to the next enabled channel
--      (round-robin, wrapping at bit 15 back to bit 0) and issue its
--      one-clock start pulse.
--   4. Repeat indefinitely until reset, a new mask write, or the mask goes
--      all-zero (scheduler goes idle, channel_done_en held low).
--
-- Notes:
--   - channel_done_en is a ONE-CLOCK start pulse per channel, not a level.
--     Each spi_adc_macro instance is assumed to latch this pulse and run
--     its own transaction FSM to completion, then assert channel_done for
--     exactly one clock in this clk domain.
--   - current_ch/next_ch are range-constrained (0 to 15) for clean
--     synthesis and simulation bounds checking.
--   - The "active" flag guards against advancing on a channel_done pulse
--     that isn't for the currently active channel (e.g. a late pulse from
--     a macro that was abandoned mid-transaction on a mask change).
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

entity rr_channel_scheduler is
    port (
        clk             : in  std_logic;
        reset           : in  std_logic;

        adc_channel_reg    : in  std_logic_vector(15 downto 0);
        adc_channel_reg_wr : in  std_logic;

        channel_done    : in  std_logic_vector(15 downto 0);
        channel_done_en : out std_logic_vector(15 downto 0)
    );
end entity rr_channel_scheduler;

architecture rtl of rr_channel_scheduler is

    signal current_ch : integer range 0 to 15 := 0;
    signal ch_en      : std_logic_vector(15 downto 0) := (others => '0');
    signal active     : std_logic := '0';

begin

    channel_done_en <= ch_en;

    process (clk)
        variable next_ch : integer range 0 to 15;
        variable found    : boolean;
    begin
        if rising_edge(clk) then
            if reset = '1' then
                current_ch <= 0;
                ch_en      <= (others => '0');
                active     <= '0';

            elsif adc_channel_reg_wr = '1' then
                -- New mask written: load first (lowest-index) enabled channel,
                -- overriding whatever cycle was previously in progress.
                ch_en  <= (others => '0');
                active <= '0';
                for i in 0 to 15 loop
                    if adc_channel_reg(i) = '1' then
                        current_ch <= i;
                        ch_en(i)   <= '1';
                        active     <= '1';
                        exit;
                    end if;
                end loop;

            elsif active = '1' and channel_done(current_ch) = '1' then
                -- Active channel finished: advance to next enabled channel,
                -- wrapping around. Ignores channel_done bits for any channel
                -- other than the one currently active.
                ch_en <= (others => '0');
                found := false;
                for i in 1 to 16 loop
                    next_ch := (current_ch + i) mod 16;
                    if adc_channel_reg(next_ch) = '1' then
                        current_ch <= next_ch;
                        ch_en(next_ch) <= '1';
                        found := true;
                        exit;
                    end if;
                end loop;
                if not found then
                    -- Mask has gone empty without a new write; go idle.
                    active <= '0';
                end if;

            else
                -- Idle / waiting for the active channel's SPI macro to finish.
                -- Deassert the one-clock start pulse; hold state otherwise.
                ch_en <= (others => '0');
            end if;
        end if;
    end process;

end architecture rtl;
