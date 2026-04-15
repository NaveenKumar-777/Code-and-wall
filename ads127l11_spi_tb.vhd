-- =============================================================================
-- Testbench: ads127l11_spi_tb
-- Simulator : Xilinx Vivado xsim (VHDL-2008)
-- =============================================================================
-- Fixes vs previous version:
--   1. to_hstring replaced with custom slv_to_hex function (Vivado compatible)
--   2. declare..begin..end blocks removed - illegal with wait statements
--   3. Integer variable timeout replaced with a plain counted wait loop
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity ads127l11_spi_tb is
end entity ads127l11_spi_tb;

architecture tb of ads127l11_spi_tb is

    -- -------------------------------------------------------------------------
    -- Constants
    -- -------------------------------------------------------------------------
    constant CLK_PERIOD : time    := 40 ns;
    constant SCLK_PROP  : time    := 15 ns;
    constant FRAME_BITS : integer := 32;

    constant ADC_WORD_1 : std_logic_vector(31 downto 0) := x"A5123456";
    constant ADC_WORD_2 : std_logic_vector(31 downto 0) := x"A5ABCDEF";
    constant REG_VAL    : std_logic_vector(7  downto 0) := x"C3";

    -- -------------------------------------------------------------------------
    -- DUT signals
    -- -------------------------------------------------------------------------
    signal clk             : std_logic := '0';
    signal rst_n           : std_logic := '0';
    signal sclk            : std_logic;
    signal cs_n            : std_logic;
    signal sdi             : std_logic;
    signal sdo             : std_logic := '0';
    signal drdy_n          : std_logic := '1';
    signal adc_data        : std_logic_vector(23 downto 0);
    signal status_byte     : std_logic_vector(7  downto 0);
    signal data_valid      : std_logic;
    signal sample_count    : std_logic_vector(31 downto 0);
    signal reg_addr        : std_logic_vector(4  downto 0) := (others => '0');
    signal reg_wdata       : std_logic_vector(7  downto 0) := (others => '0');
    signal reg_rdata       : std_logic_vector(7  downto 0);
    signal reg_rdata_valid : std_logic;
    signal reg_wr_en       : std_logic := '0';
    signal reg_rd_en       : std_logic := '0';
    signal busy            : std_logic;

    -- Slave model control
    signal slave_word   : std_logic_vector(31 downto 0) := ADC_WORD_1;
    signal sdi_capture  : std_logic_vector(31 downto 0) := (others => '0');

    -- -------------------------------------------------------------------------
    -- Helper: convert std_logic_vector to hex string
    -- Vivado xsim does not always have to_hstring in scope.
    -- -------------------------------------------------------------------------
    function slv_to_hex(slv : std_logic_vector) return string is
        constant HEX_CHARS : string(1 to 16) := "0123456789ABCDEF";
        variable padded    : std_logic_vector((((slv'length + 3) / 4) * 4) - 1 downto 0)
                             := (others => '0');
        variable result    : string(1 to (slv'length + 3) / 4);
        variable nibble    : std_logic_vector(3 downto 0);
        variable idx       : integer;
    begin
        padded(slv'length - 1 downto 0) := slv;
        for i in result'range loop
            nibble := padded(padded'length - (i-1)*4 - 1
                             downto
                             padded'length - i*4);
            idx := to_integer(unsigned(nibble));
            result(i) := HEX_CHARS(idx + 1);
        end loop;
        return result;
    end function;

    -- -------------------------------------------------------------------------
    -- Helper: wait N rising edges of clk
    -- -------------------------------------------------------------------------
    procedure wait_clk(
        signal   clk_s : in std_logic;
        constant n     : in integer) is
    begin
        for i in 1 to n loop
            wait until rising_edge(clk_s);
        end loop;
    end procedure;

begin

    -- =========================================================================
    -- Clock
    -- =========================================================================
    clk <= not clk after CLK_PERIOD / 2;

    -- =========================================================================
    -- DUT
    -- =========================================================================
    u_dut : entity work.ads127l11_spi
        generic map (FRAME_BITS => FRAME_BITS)
        port map (
            clk             => clk,
            rst_n           => rst_n,
            sclk            => sclk,
            cs_n            => cs_n,
            sdi             => sdi,
            sdo             => sdo,
            drdy_n          => drdy_n,
            adc_data        => adc_data,
            status_byte     => status_byte,
            data_valid      => data_valid,
            sample_count    => sample_count,
            reg_addr        => reg_addr,
            reg_wdata       => reg_wdata,
            reg_rdata       => reg_rdata,
            reg_rdata_valid => reg_rdata_valid,
            reg_wr_en       => reg_wr_en,
            reg_rd_en       => reg_rd_en,
            busy            => busy
        );

    -- =========================================================================
    -- ADC slave model
    -- Drives SDO MSB-first after each SCLK rising edge (CPHA=1, CPOL=0).
    -- 15 ns propagation delay modelled.
    -- =========================================================================
    p_slave : process
        variable tx_sr : std_logic_vector(31 downto 0);
        variable rx_sr : std_logic_vector(31 downto 0);
        variable b     : integer;
    begin
        sdo <= '0';
        loop
            wait until falling_edge(cs_n);
            tx_sr := slave_word;
            rx_sr := (others => '0');
            b     := 31;

            loop
                wait until rising_edge(sclk) or rising_edge(cs_n);

                if cs_n = '1' then
                    sdo <= '0';
                    exit;
                end if;

                -- Capture SDI from master
                if b >= 0 then
                    rx_sr(b) := sdi;
                end if;

                -- Drive SDO after propagation delay
                wait for SCLK_PROP;

                if b >= 0 then
                    sdo <= tx_sr(b);
                    b   := b - 1;
                else
                    sdo <= '0';
                end if;

                if b < 0 then
                    wait until rising_edge(cs_n);
                    sdo <= '0';
                    exit;
                end if;
            end loop;

            sdi_capture <= rx_sr;
        end loop;
    end process p_slave;

    -- =========================================================================
    -- Stimulus - one sequential process, no declare blocks
    -- =========================================================================
    p_stim : process
        -- busy_wait: wait up to max_cyc clock cycles for busy to go high
        -- returns immediately if busy is already high
        procedure wait_for_busy_high(max_cyc : integer) is
        begin
            for i in 1 to max_cyc loop
                if busy = '1' then
                    return;
                end if;
                wait until rising_edge(clk);
            end loop;
        end procedure;

        -- wait_for_busy_low: wait until busy falls, up to max_cyc cycles
        procedure wait_for_busy_low(max_cyc : integer) is
        begin
            for i in 1 to max_cyc loop
                if busy = '0' then
                    return;
                end if;
                wait until rising_edge(clk);
            end loop;
        end procedure;

    begin

        -- ------------------------------------------------------------------
        -- Reset
        -- ------------------------------------------------------------------
        rst_n     <= '0';
        drdy_n    <= '1';
        reg_wr_en <= '0';
        reg_rd_en <= '0';
        wait_clk(clk, 10);
        rst_n <= '1';
        wait_clk(clk, 5);

        -- ==================================================================
        -- TEST 1: ADC conversion read
        -- DRDY pulse → expect adc_data=0x123456, status=0xA5
        -- ==================================================================
        report "=== TEST 1: ADC conversion read ===" severity note;
        slave_word <= ADC_WORD_1;
        wait_clk(clk, 2);

        drdy_n <= '0';
        wait_clk(clk, 1);
        drdy_n <= '1';

        -- Wait for transaction to complete
        wait until rising_edge(clk) and data_valid = '1';
        wait_clk(clk, 1);

        assert adc_data = x"123456"
            report "TEST1 FAIL: adc_data got 0x" & slv_to_hex(adc_data) &
                   " expected 0x123456" severity error;

        assert status_byte = x"A5"
            report "TEST1 FAIL: status_byte got 0x" & slv_to_hex(status_byte) &
                   " expected 0xA5" severity error;

        assert unsigned(sample_count) = 1
            report "TEST1 FAIL: sample_count=" &
                   integer'image(to_integer(unsigned(sample_count))) &
                   " expected 1" severity error;

        report "TEST1: adc_data=0x" & slv_to_hex(adc_data) &
               "  status=0x"        & slv_to_hex(status_byte) &
               "  samples="         & integer'image(to_integer(unsigned(sample_count)))
               severity note;
        report "TEST1 PASS" severity note;

        wait_clk(clk, 10);

        -- ==================================================================
        -- TEST 2: Register write  WREG reg=0x02 data=0x7E
        -- Expected TX frame: 0x42_00_00_7E
        -- ==================================================================
        report "=== TEST 2: Register write (addr=0x02 data=0x7E) ===" severity note;
        slave_word <= ADC_WORD_2;
        wait_clk(clk, 2);

        -- Ensure not busy before issuing
        wait_for_busy_low(200);
        wait_clk(clk, 1);

        reg_addr  <= "00010";
        reg_wdata <= x"7E";
        reg_wr_en <= '1';
        wait_clk(clk, 1);
        reg_wr_en <= '0';

        -- Wait for transaction to finish
        wait_for_busy_high(10);
        wait_for_busy_low(200);
        wait_clk(clk, 2);

        -- cmd byte = 0x40 | 0x02 = 0x42
        assert sdi_capture(31 downto 24) = x"42"
            report "TEST2 FAIL: WREG cmd byte got 0x" &
                   slv_to_hex(sdi_capture(31 downto 24)) &
                   " expected 0x42" severity error;

        assert sdi_capture(7 downto 0) = x"7E"
            report "TEST2 FAIL: WREG data byte got 0x" &
                   slv_to_hex(sdi_capture(7 downto 0)) &
                   " expected 0x7E" severity error;

        assert adc_data = x"ABCDEF"
            report "TEST2 FAIL: adc_data during WREG got 0x" &
                   slv_to_hex(adc_data) &
                   " expected 0xABCDEF" severity error;

        report "TEST2: sdi_cmd=0x" & slv_to_hex(sdi_capture(31 downto 24)) &
               "  sdi_data=0x"     & slv_to_hex(sdi_capture(7 downto 0)) &
               "  adc_data=0x"     & slv_to_hex(adc_data) severity note;
        report "TEST2 PASS" severity note;

        wait_clk(clk, 10);

        -- ==================================================================
        -- TEST 3: Register read RREG reg=0x05
        -- Frame 1 TX: 0x25_00_00_00
        -- Frame 2 TX: NOP - ADC returns REG_VAL in bits[7:0]
        -- ==================================================================
        report "=== TEST 3: Register read (addr=0x05) ===" severity note;
        slave_word <= x"000000" & REG_VAL;
        wait_clk(clk, 2);

        wait_for_busy_low(200);
        wait_clk(clk, 1);

        reg_addr  <= "00101";
        reg_rd_en <= '1';
        wait_clk(clk, 1);
        reg_rd_en <= '0';

        -- Wait for reg_rdata_valid (fires after second frame)
        wait until rising_edge(clk) and reg_rdata_valid = '1';
        wait_clk(clk, 1);

        assert reg_rdata = REG_VAL
            report "TEST3 FAIL: reg_rdata got 0x" & slv_to_hex(reg_rdata) &
                   " expected 0x" & slv_to_hex(REG_VAL) severity error;

        report "TEST3: reg_rdata=0x" & slv_to_hex(reg_rdata) severity note;
        report "TEST3 PASS" severity note;

        wait_clk(clk, 10);

        -- ==================================================================
        -- TEST 4: Back-to-back DRDY - second fires mid-transaction
        -- drdy_flag must hold and FSM must re-read immediately after first
        -- ==================================================================
        report "=== TEST 4: Back-to-back DRDY (no sample loss) ===" severity note;
        slave_word <= ADC_WORD_1;
        wait_clk(clk, 2);

        wait_for_busy_low(200);

        -- First DRDY
        drdy_n <= '0';
        wait_clk(clk, 1);
        drdy_n <= '1';

        -- Wait until FSM is mid-transaction then fire second DRDY
        wait_for_busy_high(10);
        wait_clk(clk, 5);
        drdy_n <= '0';
        wait_clk(clk, 1);
        drdy_n <= '1';

        -- Wait for first data_valid
        wait until rising_edge(clk) and data_valid = '1';
        report "TEST4: first sample done, count=" &
               integer'image(to_integer(unsigned(sample_count))) severity note;

        -- FSM should pick up drdy_flag and go busy again within a few cycles
        wait_for_busy_high(10);

        assert busy = '1'
            report "TEST4 FAIL: second read not started - drdy_flag lost"
            severity error;

        -- Wait for second sample
        wait until rising_edge(clk) and data_valid = '1';
        report "TEST4: second sample done, count=" &
               integer'image(to_integer(unsigned(sample_count))) severity note;

        assert unsigned(sample_count) >= 3
            report "TEST4 FAIL: expected at least 3 total samples, got " &
                   integer'image(to_integer(unsigned(sample_count))) severity error;

        report "TEST4 PASS" severity note;
        wait_clk(clk, 10);

        -- ==================================================================
        -- TEST 5: DRDY fires during register write - must not be lost
        -- ==================================================================
        report "=== TEST 5: DRDY during register write ===" severity note;
        slave_word <= ADC_WORD_1;

        wait_for_busy_low(200);
        wait_clk(clk, 2);

        -- Start register write
        reg_addr  <= "00001";
        reg_wdata <= x"55";
        reg_wr_en <= '1';
        wait_clk(clk, 1);
        reg_wr_en <= '0';

        -- Wait until FSM is busy then fire DRDY
        wait_for_busy_high(10);
        wait_clk(clk, 3);
        drdy_n <= '0';
        wait_clk(clk, 1);
        drdy_n <= '1';

        -- Write finishes - FSM should immediately start ADC read
        wait_for_busy_low(200);
        wait_for_busy_high(10);

        assert busy = '1'
            report "TEST5 FAIL: ADC read after write not started - DRDY lost"
            severity error;

        wait until rising_edge(clk) and data_valid = '1';
        report "TEST5: ADC data=0x" & slv_to_hex(adc_data) &
               " after write" severity note;
        report "TEST5 PASS" severity note;

        wait_clk(clk, 20);

        -- ==================================================================
        report "========================================" severity note;
        report "All 5 tests complete. Check above for FAIL messages." severity note;
        report "========================================" severity note;
        std.env.stop;

    end process p_stim;

end architecture tb;