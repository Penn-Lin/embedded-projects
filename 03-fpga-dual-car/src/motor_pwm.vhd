library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity motor_pwm is
    Port ( 
        clk     : in  STD_LOGIC;
        rst_n   : in  STD_LOGIC;
        duty    : in  INTEGER range 0 to 100; -- 速度
        pwm_out : out STD_LOGIC
    );
end motor_pwm;

architecture Behavioral of motor_pwm is
    constant PERIOD : integer := 50000; -- 1kHz 频率
    signal cnt : integer range 0 to PERIOD := 0;
begin
    process(clk, rst_n)
    begin
        if rst_n = '0' then
            cnt <= 0;
            pwm_out <= '0';
        elsif rising_edge(clk) then
            if cnt < PERIOD - 1 then
                cnt <= cnt + 1;
            else
                cnt <= 0;
            end if;

            if cnt < (duty * 500) then
                pwm_out <= '1';
            else
                pwm_out <= '0';
            end if;
        end if;
    end process;
end Behavioral;