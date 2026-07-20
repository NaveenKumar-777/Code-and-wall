brary ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ads127111_spi_channel is
    port (
        clk                  : in  std_logic;
        reset                : in  std_logic;

        adc_reg_wr_en_reg    : in  std_logic_vector(15 downto 0);
        adc_reg_wr_en_reg_wr : in  std_logic;

        channel_done         : in  std_logic_vector(15 downto 0);
        channel_done_en      : out std_logic_vector(15 downto 0)
    );
end entity;

architecture rtl of ads127111_spi_channel is

    signal current_ch : integer range 0 to 15 := 0;
    signal ch_en      : std_logic_vector(15 downto 0) := (others => '0');

begin

    channel_done_en <= ch_en;

    process(clk)
        variable next_ch : integer range 0 to 15;
    begin
        if rising_edge(clk) then

            if reset = '1' then
                current_ch <= 0;
                ch_en <= (others => '0');

            ----------------------------------------------------------------
            -- Load new channel mask
            ----------------------------------------------------------------
            elsif adc_reg_wr_en_reg_wr = '1' then

                ch_en <= (others => '0');

                for i in 0 to 15 loop
                    if adc_reg_wr_en_reg(i) = '1' then
                        current_ch <= i;
                        ch_en(i) <= '1';
                        exit;
                    end if;
                end loop;

            ----------------------------------------------------------------
            -- Current channel completed
            ----------------------------------------------------------------
            elsif channel_done(current_ch) = '1' then

                ch_en <= (others => '0');

                for i in 1 to 16 loop
                    next_ch := (current_ch + i) mod 16;

                    if adc_reg_wr_en_reg(next_ch) = '1' then
                        current_ch <= next_ch;
                        ch_en(next_ch) <= '1';
                        exit;
                    end if;
                end loop;

            end if;

        end if;
    end process;

end architecture rtl;

