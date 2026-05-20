-- =============================================================================
-- fifo_rr_arbiter.vhd
-- Round-robin arbiter for 16 external FIFOs -> AXI-Stream DMA
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package fifo_pkg is
    type data_array_t is array (15 downto 0) of std_logic_vector(31 downto 0);
end package;

-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.fifo_pkg.all;

entity fifo_rr_arbiter is
    port (
        -- Clock and reset (read domain, 100 MHz)
        rd_clk   : in  std_logic;
        rst      : in  std_logic;

        -- Interface to 16 external FIFOs (rd_clk domain)
        fifo_rd_data : in  data_array_t;                      -- array(15 downto 0) of slv(31 downto 0)
        fifo_empty   : in  std_logic_vector(15 downto 0);
        fifo_rd_en   : out std_logic_vector(15 downto 0);

        -- Selection mask: bit N = 1 enables FIFO N
        sel_mask : in  std_logic_vector(15 downto 0);

        -- AXI4-Stream master output to DMA
        m_tdata  : out std_logic_vector(31 downto 0);
        m_tkeep  : out std_logic_vector(3  downto 0);
        m_tlast  : out std_logic;
        m_tvalid : out std_logic;
        m_tready : in  std_logic
    );
end entity fifo_rr_arbiter;

architecture rtl of fifo_rr_arbiter is

    -- -------------------------------------------------------------------------
    -- Arbiter state
    -- -------------------------------------------------------------------------
    type state_t is (IDLE, FETCH, STREAM);
    signal state : state_t := IDLE;

    signal cur_idx    : integer range 0 to 15 := 0;
    signal served_cnt : integer range 0 to 16 := 0;

    -- Output registers
    signal reg_tdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal reg_tlast  : std_logic := '0';
    signal reg_tvalid : std_logic := '0';
    signal reg_rd_en  : std_logic_vector(15 downto 0) := (others => '0');

    -- -------------------------------------------------------------------------
    -- Helper: count enabled FIFOs in sel_mask
    -- -------------------------------------------------------------------------
    function count_ones(v : std_logic_vector(15 downto 0))
        return integer is
        variable n : integer range 0 to 16 := 0;
    begin
        for i in 0 to 15 loop
            if v(i) = '1' then n := n + 1; end if;
        end loop;
        return n;
    end function;

    -- -------------------------------------------------------------------------
    -- Helper: find next selected non-empty FIFO after position 'from'
    -- -------------------------------------------------------------------------
    procedure find_next (
        mask   : in  std_logic_vector(15 downto 0);
        empty  : in  std_logic_vector(15 downto 0);
        from   : in  integer range 0 to 15;
        idx    : out integer range 0 to 15;
        found  : out std_logic
    ) is
        variable c : integer range 0 to 15;
    begin
        idx   := 0;
        found := '0';
        for i in 1 to 16 loop
            c := (from + i) mod 16;
            if mask(c) = '1' and empty(c) = '0' then
                idx   := c;
                found := '1';
                exit;
            end if;
        end loop;
    end procedure;

    signal any_valid : std_logic;

begin

    -- =========================================================================
    -- Combinatorial: any selected FIFO non-empty?
    -- =========================================================================
    process(sel_mask, fifo_empty)
        variable v : std_logic;
    begin
        v := '0';
        for i in 0 to 15 loop
            if sel_mask(i) = '1' and fifo_empty(i) = '0' then
                v := '1';
            end if;
        end loop;
        any_valid <= v;
    end process;

    -- =========================================================================
    -- Round-robin FSM
    -- =========================================================================
    process(rd_clk)
        variable nxt_idx    : integer range 0 to 15;
        variable nxt_found  : std_logic;
        variable new_served : integer range 0 to 16;
        variable sel_cnt    : integer range 0 to 16;
        variable is_last    : std_logic;
    begin
        if rising_edge(rd_clk) then
            reg_rd_en <= (others => '0');

            if rst = '1' then
                state      <= IDLE;
                cur_idx    <= 0;
                served_cnt <= 0;
                reg_tvalid <= '0';
                reg_tlast  <= '0';
                reg_tdata  <= (others => '0');

            else
                case state is

                    when IDLE =>
                        reg_tvalid <= '0';
                        reg_tlast  <= '0';
                        served_cnt <= 0;

                        if any_valid = '1' then
                            find_next(sel_mask, fifo_empty, 15,
                                      nxt_idx, nxt_found);
                            if nxt_found = '1' then
                                reg_rd_en(nxt_idx) <= '1';
                                cur_idx            <= nxt_idx;
                                state              <= FETCH;
                            end if;
                        end if;

                    when FETCH =>
                        state <= STREAM;

                    when STREAM =>
                        -- fifo_rd_data is now a direct array index - clean and simple
                        reg_tdata  <= fifo_rd_data(cur_idx);
                        reg_tvalid <= '1';

                        if reg_tvalid = '1' and m_tready = '1' then

                            sel_cnt    := count_ones(sel_mask);
                            new_served := served_cnt + 1;

                            if new_served >= sel_cnt then
                                is_last    := '1';
                                new_served := 0;
                            else
                                is_last := '0';
                            end if;

                            reg_tlast  <= is_last;
                            served_cnt <= new_served;

                            find_next(sel_mask, fifo_empty, cur_idx,
                                      nxt_idx, nxt_found);

                            if nxt_found = '1' then
                                reg_rd_en(nxt_idx) <= '1';
                                cur_idx            <= nxt_idx;
                            else
                                reg_tvalid <= '0';
                                state      <= IDLE;
                            end if;

                        end if;

                end case;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Output assignments
    -- =========================================================================
    fifo_rd_en <= reg_rd_en;
    m_tdata    <= reg_tdata;
    m_tkeep    <= "1111";
    m_tlast    <= reg_tlast;
    m_tvalid   <= reg_tvalid;

end architecture rtl;