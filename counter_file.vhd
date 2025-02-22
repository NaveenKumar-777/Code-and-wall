----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 02/22/2025 07:10:43 AM
-- Design Name: 
-- Module Name: counter_file - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;
use IEEE.STD_LOGIC_UNSIGNED.all;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity counter_file is
  Port (
        clk : std_logic;
        reset : std_logic );
end counter_file;

architecture Behavioral of counter_file is

type const_array is array (0 to 39) of STD_LOGIC_VECTOR(15 downto 0);
signal default_val : const_array :=
(
(0) => x"0000",
(1) => x"0001",
(2) => x"0002",
(3) => x"0003",
(4) => x"0004",
(5) => x"0005",
(6) => x"0006",
(7) => x"0007",
(8) => x"0008",
(9) => x"0009",
(10) => x"000A",
(11) => x"000B",
(12) => x"000C",
(13) => x"000D",
(14) => x"000E",
(15) => x"000F",
(16) => x"0010",
(17) => x"0011",
(18) => x"0012",
(19) => x"0013",
(20) => x"0014",
(21) => x"0015",
(22) => x"0016",
(23) => x"0017",
(24) => x"0018",
(25) => x"0019",
(26) => x"001A",
(27) => x"001B",
(28) => x"001C",
(29) => x"001D",
(30) => x"001E",
(31) => x"001F",
(32) => x"0020",
(33) => x"0021",
(34) => x"0022",
(35) => x"0023",
(36) => x"0024",
(37) => x"0025",
(38) => x"0026",
(39) => x"0027"
);

signal off_time : std_logic_vector(31 downto 0):=x"00000100";
signal counter : std_logic_vector(31 downto 0) := (others=>'0');
signal mod_out,mon_reg,rising_pul,falling_pul,start,ending : std_logic :='0';
signal dac_counter : integer range 0 to 40 := 0;
signal dac_out : std_logic_vector(15 downto 00) := (others=>'0');

component pulse_gen
port(
            clk    : in  STD_LOGIC;        -- Clock input
           din    : in  STD_LOGIC;        -- Input signal (d1)
           dout   : out STD_LOGIC         -- Output single pulse (dout)
);
end component;

begin

process(clk)
begin
if(rising_edge(clk)) then

if(off_time >= counter) then
    mod_out <= '1'; 
    counter <= counter + 1;
else 
    mod_out <= '0'; 
    counter <= (others=>'0');
end if;

if(counter <= off_time - x"28") then
    mon_reg <= '1';
else 
    mon_reg <= '0'; 
end if;
end if;

end process;

v2 : pulse_gen
port map(
 clk  => clk,
din   => mod_out,
dout  => rising_pul
);

v1 : pulse_gen
port map(
 clk  => clk,
din   => mon_reg,
dout  => falling_pul
);

process(clk)
begin
if(rising_pul = '1') then
    start <= '1';
end if;

if(falling_pul = '1')then
    ending <= '1';
end if;

if(rising_edge(clk)) then

if(start = '1') and (dac_counter < 39)  then
    dac_counter <= dac_counter + 1;
 else
    start <= '0';
 end if;
 
 if(ending = '1') and (dac_counter > 0) then
    dac_counter <= dac_counter - 1;
 else
    ending <= '0';
 end if;

end if;
end process;

dac_out <= default_val(dac_counter);


end Behavioral;
