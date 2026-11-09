library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity sensor_scanner is
    Generic (
        STABLE_TIME : integer := 5000  -- 100us等待，保证信号稳定
    );
    Port ( 
        clk        : in  STD_LOGIC;
        rst_n      : in  STD_LOGIC;
        sensor_out : in  STD_LOGIC;                     -- 接 OUT
        ad_pins    : out STD_LOGIC_VECTOR (2 downto 0); -- 接 AD2, AD1, AD0
        data_ready : out STD_LOGIC_VECTOR (7 downto 0)  -- 输出 [7..0] 数据
    );
end sensor_scanner;

architecture Behavioral of sensor_scanner is
    signal channel_idx : integer range 0 to 7 := 0;
    signal delay_cnt   : integer range 0 to STABLE_TIME := 0;
    signal temp_data   : STD_LOGIC_VECTOR (7 downto 0) := (others => '0');
begin
    process(clk, rst_n)
    begin
        if rst_n = '0' then
            channel_idx <= 0;
            delay_cnt <= 0;
            ad_pins <= "000";
            data_ready <= (others => '0');
        elsif rising_edge(clk) then
            -- 输出地址
            ad_pins <= std_logic_vector(to_unsigned(channel_idx, 3));
            
            if delay_cnt < STABLE_TIME then
                delay_cnt <= delay_cnt + 1;
            else
                -- 采样: 1=黑, 0=白 (根据你刚才的测试结果)
                temp_data(channel_idx) <= sensor_out;
                
                delay_cnt <= 0;
                if channel_idx = 7 then
                    channel_idx <= 0;
                    data_ready <= temp_data; -- 更新一帧数据
                else
                    channel_idx <= channel_idx + 1;
                end if;
            end if;
        end if;
    end process;
end Behavioral;