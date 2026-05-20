-- =============================================================================
-- tb_fifo_rr_arbiter.vhd
-- Testbench for fifo_rr_arbiter
-- Vivado 2021.2 compatible (VHDL-2008)
--
-- Test cases:
--   TC1 : 2 FIFOs selected (FIFO 1 & FIFO 3)
--         Data word = [3:0] channel_id & x"0" & 24-bit counter
--   TC2 : All 16 FIFOs selected
--         Same data format per channel
--
-- Data word format (32-bit):
--   [31:28] = channel number (4-bit)
--   [27:24] = x"0"
--   [23:0]  = 24-bit counter (per-channel, increments each word)
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.fifo_pkg.all;

entity tb_fifo_rr_arbiter is
end entity tb_fifo_rr_arbiter;

architecture sim of tb_fifo_rr_arbiter is

    constant CLK_PERIOD   : time    := 10 ns;
    constant FIFO_DEPTH   : integer := 16;
    constant NUM_CHANNELS : integer := 16;

    -- Custom array types (integer_vector not available in Vivado 2021.2)
    type int_array_t     is array (0 to NUM_CHANNELS-1) of integer range 0 to FIFO_DEPTH;
    type fifo_mem_t      is array (0 to FIFO_DEPTH-1)   of std_logic_vector(31 downto 0);
    type all_fifo_mem_t  is array (0 to NUM_CHANNELS-1) of fifo_mem_t;

    signal rd_clk        : std_logic := '0';
    signal rst           : std_logic := '1';
    signal fifo_rd_data  : data_array_t := (others => (others => '0'));
    signal fifo_empty    : std_logic_vector(15 downto 0) := (others => '1');
    signal fifo_rd_en    : std_logic_vector(15 downto 0);
    signal sel_mask      : std_logic_vector(15 downto 0) := (others => '0');
    signal m_tdata       : std_logic_vector(31 downto 0);
    signal m_tkeep       : std_logic_vector(3  downto 0);
    signal m_tlast       : std_logic;
    signal m_tvalid      : std_logic;
    signal m_tready      : std_logic := '1';
    signal fifo_mem      : all_fifo_mem_t := (others => (others => (others => '0')));
shared variable fifo_rd_ptr : int_array_t := (others => 0);
shared variable fifo_wr_ptr : int_array_t := (others => 0);
    signal sim_done      : std_logic := '0';

    function make_word(ch : integer; cnt : integer)
        return std_logic_vector is
        variable w : std_logic_vector(31 downto 0);
    begin
        w(31 downto 28) := std_logic_vector(to_unsigned(ch,  4));
        w(27 downto 24) := x"0";
        w(23 downto  0) := std_logic_vector(to_unsigned(cnt, 24));
        return w;
    end function;

    -- Replaces to_hstring (not in Vivado 2021.2)
    function to_hex_str(v : std_logic_vector) return string is
        constant hex_chars : string(1 to 16) := "0123456789ABCDEF";
        variable padded    : std_logic_vector((((v'length+3)/4)*4)-1 downto 0) := (others => '0');
        variable result    : string(1 to (v'length+3)/4);
        variable nibble    : std_logic_vector(3 downto 0);
        variable idx       : integer;
    begin
        padded(v'length-1 downto 0) := v;
        for i in result'range loop
            idx    := result'length - i;
            nibble := padded(idx*4-1 downto idx*4-4);
            result(i) := hex_chars(to_integer(unsigned(nibble)) + 1);
        end loop;
        return result;
    end function;

begin

    -- =========================================================================
    -- Clock
    -- =========================================================================
    rd_clk <= not rd_clk after CLK_PERIOD / 2 when sim_done = '0' else '0';

    -- =========================================================================
    -- DUT
    -- =========================================================================
    u_dut : entity work.fifo_rr_arbiter
        port map (
            rd_clk       => rd_clk,
            rst          => rst,
            fifo_rd_data => fifo_rd_data,
            fifo_empty   => fifo_empty,
            fifo_rd_en   => fifo_rd_en,
            sel_mask     => sel_mask,
            m_tdata      => m_tdata,
            m_tkeep      => m_tkeep,
            m_tlast      => m_tlast,
            m_tvalid     => m_tvalid,
            m_tready     => m_tready
        );

    -- =========================================================================
    -- Simulated FIFO behaviour
    -- Drives fifo_rd_data and fifo_empty based on rd_en and internal pointers
    -- =========================================================================
fifo_model : process(rd_clk)
begin
    if rising_edge(rd_clk) then
        for ch in 0 to NUM_CHANNELS-1 loop

            fifo_rd_data(ch) <= fifo_mem(ch)(fifo_rd_ptr(ch));

            if fifo_rd_en(ch) = '1' and fifo_empty(ch) = '0' then
                if fifo_rd_ptr(ch) = fifo_wr_ptr(ch) - 1 then
                    fifo_empty(ch)          <= '1';
                    fifo_rd_ptr(ch) := fifo_rd_ptr(ch) + 1;  -- := not <=
                else
                    fifo_rd_ptr(ch) := fifo_rd_ptr(ch) + 1;  -- := not <=
                end if;
            end if;

        end loop;
    end if;
end process;

    -- =========================================================================
    -- Stimulus
    -- =========================================================================
    stim : process

        -- ---------------------------------------------------------------------
        -- Procedure: load N words into a single FIFO channel
        -- ---------------------------------------------------------------------
procedure load_fifo(ch : integer; n_words : integer) is
begin
    for w in 0 to n_words-1 loop
        fifo_mem(ch)(w) <= make_word(ch, w);
    end loop;
    fifo_wr_ptr(ch) := n_words;   -- := not <=
    fifo_rd_ptr(ch) := 0;         -- := not <=
    fifo_empty(ch)  <= '0';
    wait for 1 ns;
end procedure;

        -- ---------------------------------------------------------------------
        -- Procedure: wait for N clock rising edges
        -- ---------------------------------------------------------------------
        procedure clk_wait(n : integer) is
        begin
            for i in 1 to n loop
                wait until rising_edge(rd_clk);
            end loop;
        end procedure;

        -- ---------------------------------------------------------------------
        -- Procedure: run until m_tvalid drops (arbiter returns to IDLE)
        -- with optional TREADY toggling to test backpressure
        -- ---------------------------------------------------------------------
        procedure drain_stream(test_name : string; backpressure : boolean) is
            variable timeout : integer := 0;
        begin
            report "=== " & test_name & " : streaming started ===" severity note;
            while m_tvalid = '1' or timeout < 5 loop
                wait until rising_edge(rd_clk);
                timeout := timeout + 1;

                if m_tvalid = '1' then
                    timeout := 0;
                    -- Print each transferred word
                    if m_tready = '1' then
                        report "  TDATA=0x" &
                               to_hex_str(m_tdata) &
                               "  CH="  & integer'image(to_integer(unsigned(m_tdata(31 downto 28)))) &
                               "  CNT=" & integer'image(to_integer(unsigned(m_tdata(23 downto  0)))) &
                               "  TLAST=" & std_logic'image(m_tlast)
                               severity note;
                    end if;

                    -- Toggle TREADY to exercise backpressure
                    if backpressure then
                        m_tready <= not m_tready;
                    end if;
                end if;

                if timeout >= 200 then
                    report "TIMEOUT waiting for stream to end" severity failure;
                    exit;
                end if;
            end loop;
            report "=== " & test_name & " : streaming done ===" severity note;
        end procedure;

    begin
        -- -----------------------------------------------------------------------
        -- Reset
        -- -----------------------------------------------------------------------
        rst      <= '1';
        sel_mask <= (others => '0');
        m_tready <= '1';
        clk_wait(5);
        rst <= '0';
        clk_wait(2);

        -- =======================================================================
        -- TEST CASE 1: 2 FIFOs selected - FIFO 1 and FIFO 3
        -- Expected stream: FIFO1[0], FIFO3[0], FIFO1[1], FIFO3[1], ...
        -- =======================================================================
        report "########################################" severity note;
        report "# TC1: FIFO 1 & 3 selected, no backpressure" severity note;
        report "########################################" severity note;

        -- Load FIFO_DEPTH words into channel 1 and channel 3
        load_fifo(1, FIFO_DEPTH);
        load_fifo(3, FIFO_DEPTH);

        -- Enable only FIFO 1 and FIFO 3
        sel_mask <= "0000000000001010";  -- bit1=FIFO1, bit3=FIFO3

        clk_wait(2);
        drain_stream("TC1 no-backpressure", false);
        clk_wait(4);

        -- =======================================================================
        -- TEST CASE 2: 2 FIFOs, FIFO 1 & 3, WITH backpressure (TREADY toggling)
        -- =======================================================================
        report "########################################" severity note;
        report "# TC2: FIFO 1 & 3 selected, WITH backpressure" severity note;
        report "########################################" severity note;

        -- Reload FIFOs
        load_fifo(1, FIFO_DEPTH);
        load_fifo(3, FIFO_DEPTH);

        sel_mask <= "0000000000001010";
        m_tready <= '1';
        clk_wait(2);
        drain_stream("TC2 backpressure", true);
        m_tready <= '1';
        clk_wait(4);

        -- =======================================================================
        -- TEST CASE 3: ALL 16 FIFOs selected
        -- Expected: FIFO0[0]..FIFO15[0], FIFO0[1]..FIFO15[1], ...
        -- =======================================================================
        report "########################################" severity note;
        report "# TC3: All 16 FIFOs selected, no backpressure" severity note;
        report "########################################" severity note;

        -- Load all 16 channels
        for ch in 0 to NUM_CHANNELS-1 loop
            load_fifo(ch, FIFO_DEPTH);
        end loop;

        sel_mask <= (others => '1');   -- all 16
        m_tready <= '1';
        clk_wait(2);
        drain_stream("TC3 all-FIFOs", false);
        clk_wait(4);

        -- =======================================================================
        -- TEST CASE 4: All 16 FIFOs WITH backpressure
        -- =======================================================================
        report "########################################" severity note;
        report "# TC4: All 16 FIFOs selected, WITH backpressure" severity note;
        report "########################################" severity note;

        for ch in 0 to NUM_CHANNELS-1 loop
            load_fifo(ch, FIFO_DEPTH);
        end loop;

        sel_mask <= (others => '1');
        m_tready <= '1';
        clk_wait(2);
        drain_stream("TC4 all-FIFOs backpressure", true);
        m_tready <= '1';
        clk_wait(4);

        -- =======================================================================
        -- Done
        -- =======================================================================
        report "########################################" severity note;
        report "# ALL TEST CASES COMPLETE" severity note;
        report "########################################" severity note;
        sim_done <= '1';
        wait;
    end process;

end architecture sim;