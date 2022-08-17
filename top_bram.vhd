--
-- Copyright (c) 2015 Marko Zec, University of Zagreb
-- All rights reserved.
--
-- Redistribution and use in source and binary forms, with or without
-- modification, are permitted provided that the following conditions
-- are met:
-- 1. Redistributions of source code must retain the above copyright
--    notice, this list of conditions and the following disclaimer.
-- 2. Redistributions in binary form must reproduce the above copyright
--    notice, this list of conditions and the following disclaimer in the
--    documentation and/or other materials provided with the distribution.
--
-- THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
-- ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
-- IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
-- ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
-- FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
-- DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
-- OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
-- HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
-- LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
-- OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
-- SUCH DAMAGE.
--
-- $Id$
--

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

library unisim;
use unisim.vcomponents.all;

use work.f32c_pack.all;


entity glue is
    generic (
	-- ISA
	C_arch: integer := ARCH_MI32;

	-- Main clock: N * 10 MHz
	C_clk_freq: integer := 100;

	-- SoC configuration options
	C_bram_size: integer := 64
    );
    port (
	clk: in std_logic; -- 100 MHz
	RsTx: out std_logic; -- FTDI UART
	RsRx: in std_logic; -- FTDI UART
	JA, JB, JC: inout std_logic_vector(7 downto 0); -- PMODs
	seg: out std_logic_vector(6 downto 0); -- 7-segment display
	dp: out std_logic; -- 7-segment display
	an: out std_logic_vector(3 downto 0); -- 7-segment display
	led: out std_logic_vector(15 downto 0);
	sw: in std_logic_vector(15 downto 0);
	btnC, btnU, btnD, btnL, btnR: in std_logic
    );
end glue;

architecture Behavioral of glue is
    signal rs232_break1: std_logic;
    signal rs232_break2: std_logic;
    signal btns1: std_logic_vector(15 downto 0);
    signal btns2: std_logic_vector(15 downto 0);
    signal lcd_7seg: std_logic_vector(15 downto 0);
	signal gpio_s_4: std_logic_vector(3 downto 0);
	signal gpio_s_104: std_logic_vector(103 downto 0);
	 signal RsTx1: std_logic;
	 signal RsTx2: std_logic;
	 signal RsRx1: std_logic;
	 signal RsRx2: std_logic;
	 signal JA1, JB1, JC1: std_logic_vector(7 downto 0);
	 signal JA2, JB2, JC2: std_logic_vector(7 downto 0);
	 signal seg1: std_logic_vector(6 downto 0);
	 signal seg2: std_logic_vector(6 downto 0);
	 signal dp1: std_logic;
	 signal dp2: std_logic;
	 signal an1: std_logic_vector(3 downto 0);
	 signal an2: std_logic_vector(3 downto 0);
	 signal led1: std_logic_vector(15 downto 0);
	 signal led2: std_logic_vector(15 downto 0);
	 signal sw1: std_logic_vector(15 downto 0);
	 signal sw2: std_logic_vector(15 downto 0);
	 signal btnC1, btnU1, btnD1, btnL1, btnR1: std_logic;
	 signal btnC2, btnU2, btnD2, btnL2, btnR2: std_logic;
begin
    
    led(15 downto 8) <= led1(7 downto 0);
	led(7 downto 0) <= led2(7 downto 0);
	process(clk) begin
	   if rising_edge (clk) then
	       if sw(0) = '1' then
			 RsTx <= RsTx1;
			RsRx1 <= RsRx;
			--JA1 <= "00000000";
			--JB1 <= "00000000";
			--JC1 <= "00000000";
			seg <= seg1;
			dp <= dp1;
			an <= an1;
--			led <= led1;
			sw1 <= sw;
			btnC1 <= btnC;
			btnU1 <= btnU;
			btnD1 <= btnD;
			btnL1 <= btnL;
			btnR1 <= btnR;
			
			RsRx2 <= '0';
			--JA2 <= "00000000";
			--JB2 <= "00000000";
			--JC2 <= "00000000";--(others => '0')
			--sw2 <= "0000000000000000";
			btnC2 <= '0';
			btnU2 <= '0';
			btnD2 <= '0';
			btnL2 <= '0';
			btnR2 <= '0';
		else
			RsTx <= RsTx2;
			RsRx2 <= RsRx;
			--JA2 <= "00000000";
			--JB2 <= "00000000";
			--JC2 <= "00000000";
			seg <= seg2;
			dp <= dp2;
			an <= an2;
--			led <= led2;
			sw2 <= sw;
			btnC2 <= btnC;
			btnU2 <= btnU;
			btnD2 <= btnD;
			btnL2 <= btnL;
			btnR2 <= btnR;
			
			RsRx1 <= '0';
			--JA1 <= "00000000";
			--JB1 <= "00000000";
			--JC1 <= "00000000";
			--sw1 <= "0000000000000000";
			btnC1 <= '0';
			btnU1 <= '0';
			btnD1 <= '0';
			btnL1 <= '0';
			btnR1 <= '0';
		end if;
		end if;
	end process;


---Bu kod xfnfzd
    -- generic BRAM glue 1
    glue_bram1: entity work.glue_bram
    generic map (
	C_clk_freq => C_clk_freq,
	C_arch => C_arch,
	C_bram_size => C_bram_size
    )
    port map (
	clk => clk,
	sio_txd(0) => RsTx1, sio_rxd(0) => RsRx1, sio_break(0) => rs232_break1,
	gpio(7 downto 0) => ja1, gpio(15 downto 8) => jb1,
	gpio(23 downto 16) => jc1, gpio(127 downto 24) => (gpio_s_104),
	simple_out(15 downto 0) => led1, simple_out(22 downto 16) => seg1,
	simple_out(23) => dp1, simple_out(27 downto 24) => an1,
	simple_out(31 downto 28) => (gpio_s_4),
	simple_in(15 downto 0) => btns1, simple_in(31 downto 16) => sw1,
	spi_miso => (others => '0')
    );
    btns1 <= x"00" & "000" & btnc1 & btnu1 & btnd1 & btnl & btnr;

    res1: startupe2
    generic map (
	prog_usr => "FALSE"
    )
    port map (
	clk => clk,
	gsr => rs232_break1,
	gts => '0',
	keyclearb => '0',
	pack => '1',
	usrcclko => clk,
	usrcclkts => '0',
	usrdoneo => '1',
	usrdonets => '0'
    );
	 
	 
	 -- generic BRAM glue 2
    glue_bram2: entity work.glue_bram
    generic map (
	C_clk_freq => C_clk_freq,
	C_arch => C_arch,
	C_bram_size => C_bram_size
    )
    port map (
	clk => clk,
	sio_txd(0) => rstx2, sio_rxd(0) => rsrx2, sio_break(0) => rs232_break2,
	gpio(7 downto 0) => ja2, gpio(15 downto 8) => jb2,
	gpio(23 downto 16) => jc2, gpio(127 downto 24) => (gpio_s_104),
	simple_out(15 downto 0) => led2, simple_out(22 downto 16) => seg2,
	simple_out(23) => dp2, simple_out(27 downto 24) => an2,
	simple_out(31 downto 28) => (gpio_s_4),
	simple_in(15 downto 0) => btns2, simple_in(31 downto 16) => sw2,
	spi_miso => (others => '0')
    );
    btns2 <= x"00" & "000" & btnc2 & btnu2 & btnd2 & btnl & btnr;

--    res2: startupe2
--    generic map (
--	prog_usr => "FALSE"
--    )
--    port map (
--	clk => clk,
--	gsr => rs232_break2,
--	gts => '0',
--	keyclearb => '0',
--	pack => '1',
--	usrcclko => clk,
--	usrcclkts => '0',
--	usrdoneo => '1',
--	usrdonets => '0'
--    );

end Behavioral;
