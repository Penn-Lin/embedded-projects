library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity hc_sr04 is
    Port ( 
        clk         : in  STD_LOGIC; -- 50MHz
        rst_n       : in  STD_LOGIC;
        echo        : in  STD_LOGIC;
        trig        : out STD_LOGIC;
        stop_flag   : out STD_LOGIC;  -- 保留接口（跟踪逻辑内不再使用）
        distance_mm : out INTEGER range 0 to 5000 -- 距离输出(mm)
    );
end hc_sr04;

architecture Behavioral of hc_sr04 is
    signal trig_cnt : integer range 0 to 1000000 := 0; -- 20ms周期
    signal echo_cnt : integer range 0 to 5000000 := 0;
    signal echo_sync : STD_LOGIC_VECTOR(1 downto 0) := (others => '0');
    constant CLK_PER_MM : integer := 294; -- 1mm对应294个50MHz时钟周期
    
begin
    process(clk, rst_n)
    begin
        if rst_n = '0' then
            trig <= '0';
            stop_flag <= '0';
            trig_cnt <= 0;
            echo_cnt <= 0;
            echo_sync <= (others => '0');
            distance_mm <= 0;
            
        elsif rising_edge(clk) then
            -- 1. 产生Trig信号：20ms周期，20us高电平
            if trig_cnt < 1000000 then
                trig_cnt <= trig_cnt + 1;
                -- 修复：进程内用if替换when
                if trig_cnt < 1000 then
                    trig <= '1';
                else
                    trig <= '0';
                end if;
            else
                trig_cnt <= 0;
            end if;
            
            -- 2. Echo信号同步防抖（仅两级，无延迟）
            echo_sync(0) <= echo;
            echo_sync(1) <= echo_sync(0);
            
            -- 3. 测量Echo宽度（实时计算，无滤波）
            if trig_cnt = 1000 then -- Trig结束后开始计数
                echo_cnt <= 0;
            elsif echo_sync(1) = '1' then
                echo_cnt <= echo_cnt + 1;
            else
                -- 实时输出距离（无延迟）
                distance_mm <= echo_cnt / CLK_PER_MM;
                -- 修复：进程内用if替换when
                if (echo_cnt / CLK_PER_MM) < 60 then
                    stop_flag <= '1';
                else
                    stop_flag <= '0';
                end if;
            end if;
            
        end if;
    end process;
end Behavioral;