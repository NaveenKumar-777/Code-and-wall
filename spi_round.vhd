process(clk)
    variable next_ch : integer;
begin
    if rising_edge(clk) then

        if reset = '1' then
            channel_done_en <= (others => '0');
            current_ch <= 0;

        -- Load new mask
        elsif adc_reg_wr_en_reg_wr = '1' then

            for i in 0 to 15 loop
                if adc_reg_wr_en_reg(i) = '1' then
                    current_ch <= i;
                    channel_done_en <= (others => '0');
                    channel_done_en(i) <= '1';
                    exit;
                end if;
            end loop;

        -- Current channel finished
        elsif channel_done(current_ch) = '1' then

            for i in 1 to 16 loop
                next_ch := (current_ch + i) mod 16;

                if adc_reg_wr_en_reg(next_ch) = '1' then
                    current_ch <= next_ch;
                    channel_done_en <= (others => '0');
                    channel_done_en(next_ch) <= '1';
                    exit;
                end if;
            end loop;

        end if;
    end if;
end process;
:wq

