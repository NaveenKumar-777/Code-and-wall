----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 10/08/2023 08:34:20 AM
-- Design Name: 
-- Module Name: vending_mach - Behavioral
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
use IEEE.std_logic_unsigned.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity vending_mach is
  Port ( 
        clk       : in std_logic;
        rst       : in std_logic := '0';
        start     : in std_logic := '0';
        coin_inp  : in std_logic := '0';
        enter     : in std_logic := '0';
        finish    : in std_logic := '0';
        button_1  : in std_logic := '0';
        button_2  : in std_logic := '0';
        button_3  : in std_logic := '0';
        prod1_mot : out std_logic := '0'; 
        prod2_mot : out std_logic := '0'; 
        prod3_mot : out std_logic := '0';
        total     : out std_logic_vector (5 downto 0) := (others => '0');
        balance   : out std_logic_vector (5 downto 0) := (others => '0')
        );
        
end vending_mach;

architecture Behavioral of vending_mach is

type state is (coin_input, reset, product_state, fini);
signal current_state                            : state;
signal coin_inp_reg, total_reg                  : std_logic_vector (5 downto 0) := (others => '0'); 
signal count1, count2, count3                   : std_logic_vector (2 downto 0) := (others => '0');
signal button_1_reg, button_2_reg, button_3_reg : std_logic := '0'; 


begin

process (clk)
begin
    if (rst = '1') then
        current_state <= reset;
    end if;
    if (start = '1') then
        current_state <= coin_input;
    end if;
    if ((button_1 = '1') or (button_2 = '1') or (button_3 = '1')) then
        current_state <= product_state;
    end if;
    if (finish = '1') then
        current_state <= fini;
    end if;
    
end process;


process(clk)
begin
    case current_state is
        when reset =>
            total_reg <= (others =>'0');
        when coin_input => 
            if rising_edge(clk) then
                if (coin_inp = '1') then
                    coin_inp_reg <= coin_inp_reg + 1;
                elsif (enter = '1') then
                    total_reg <= coin_inp_reg;
                end if;    
            end if;
         
         when product_state =>
            if rising_edge(clk) then
                if (button_1 = '1') and (total_reg >= 8) then
                    button_1_reg <= '1';
                    total_reg <= total_reg - 8;
                end if;
            
                if (button_2 = '1') and (total_reg >= 5) then
                    button_2_reg <= '1';
                    total_reg <= total_reg - 5;
                end if;
                if (button_3 = '1') and (total_reg >= 1) then
                    button_3_reg <= '1';
                    total_reg <= total_reg - 1;
                end if;
            end if;
             
             
            if (button_1_reg = '1') then
                if (count1 = 5) then
                    button_1_reg <= '0';
                    count1 <= (others => '0');
                else
                    count1 <= count1 + 1;
                end if;
            end if;
            
            if (button_2_reg = '1') then
                if (count2 = 5) then
                    button_2_reg <= '0';
                    count2 <= (others => '0');
                else
                    count2 <= count2 + 1;
                end if;
            end if;
            
            if (button_3_reg = '1') then
                if (count3 = 5) then
                    button_3_reg <= '0';
                    count3 <= (others => '0');
                else
                    count3 <= count3 + 1;
                end if;
            end if;
            
            when fini => 
                if rising_edge(clk) then
                    balance <= total_reg;
                    total_reg <= (others => '0');
                    coin_inp_reg <= (others => '0');
                end if;    
    end case; 
end process;


process(clk) 
begin
        if rising_edge(clk) then
            if (button_1_reg = '1') then
                    prod1_mot <= '1';
            elsif (button_1_reg = '0') then
                    prod1_mot <= '0';
            end if;
        end if;
        
        if rising_edge(clk) then
            if (button_2_reg = '1') then
                    prod2_mot <= '1';
              elsif (button_2_reg = '0') then
                    prod2_mot <= '0';
            end if;
        end if;
        
        if rising_edge(clk) then
            if (button_3_reg = '1') then
                    prod3_mot <= '1';
             elsif (button_3_reg = '0') then
                    prod3_mot <= '0';
            end if;
        end if;   
end process;

total <= total_reg;

end Behavioral;










--add_force {/vending_mach/clk} -radix hex {1 0ns} {0 50000ps} -repeat_every 100000ps
--add_force {/vending_mach/start} -radix hex {1 0ns}
--run 100 ns
--add_force {/vending_mach/start} -radix hex {0 0ns}
--run 100 ns
--add_force {/vending_mach/coin_inp} -radix hex {1 0ns}
--run 1600 ns
--add_force {/vending_mach/coin_inp} -radix hex {0 0ns}
--add_force {/vending_mach/enter} -radix hex {1 0ns}
--run 100 ns
--add_force {/vending_mach/enter} -radix hex { 0 0ns}
--add_force {/vending_mach/button_1} -radix hex {1 0ns}
--run 100 ns
--add_force {/vending_mach/button_1} -radix hex {0 0ns}
--add_force {/vending_mach/button_2} -radix hex {1 0ns}
--run 100 ns
--add_force {/vending_mach/button_2} -radix hex {0 0ns}
--add_force {/vending_mach/button_3} -radix hex {1 0ns}
--run 100 ns
--add_force {/vending_mach/button_3} -radix hex {0 0ns}
--run 400 ns
--add_force {/vending_mach/finish} -radix hex {1 0ns}
--run 100 ns
--add_force {/vending_mach/finish} -radix hex {0 0ns}
--run 200 ns