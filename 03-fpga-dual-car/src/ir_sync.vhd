library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity ir_sync is
    Port ( 
        clk        : in  STD_LOGIC; -- 50MHz
        rst_n      : in  STD_LOGIC;
        
        -- 发送接口 (领头车用)
        tx_trigger : in  STD_LOGIC; -- 输入高电平脉冲，开始发送
        ir_tx_pin  : out STD_LOGIC; -- 接红外发射管
        
        -- 接收接口 (跟随车用)
        ir_rx_pin  : in  STD_LOGIC; -- 接红外接收头 (VS1838)
        rx_detect  : out STD_LOGIC  -- 输出高电平脉冲，表示收到信号
    );
end ir_sync;

architecture Behavioral of ir_sync is

    -- === 发送参数 ===
    -- 38kHz 载波: 50MHz / 38000 ≈ 1315. 翻转周期 ≈ 657
    constant CARRIER_HALF : integer := 657;
    signal carrier_cnt    : integer range 0 to 1000 := 0;
    signal carrier_38k    : std_logic := '0';
    
    -- 发送状态机: 发送 9ms 载波 -> 4.5ms 空闲
    -- 9ms = 450,000 载波周期(太难算)，直接用时钟计数
    -- 50MHz: 9ms = 450,000; 实际上 9ms = 450,000个周期? 不，是 50*10^6 * 0.009 = 450,000
    constant TX_TIME_9MS  : integer := 450000; 
    constant TX_TIME_4MS5 : integer := 225000;
    
    type tx_state_t is (IDLE, SEND_9MS, SEND_GAP, DONE);
    signal tx_state : tx_state_t := IDLE;
    signal tx_cnt   : integer := 0;

    -- === 接收参数 ===
    -- VS1838 输出低电平表示收到载波
    -- 我们检测：低电平持续 8ms~10ms，紧接着高电平持续 > 2ms
    signal rx_cnt     : integer := 0;
    signal rx_sync_0  : std_logic := '1';
    signal rx_sync_1  : std_logic := '1'; -- 打两拍消除亚稳态
    type rx_state_t is (RX_IDLE, RX_CHECK_LOW, RX_CHECK_HIGH);
    signal rx_state   : rx_state_t := RX_IDLE;

begin

    -- =============================================================
    -- 1. 38kHz 载波生成器
    -- =============================================================
    process(clk, rst_n)
    begin
        if rst_n = '0' then
            carrier_cnt <= 0;
            carrier_38k <= '0';
        elsif rising_edge(clk) then
            if carrier_cnt < CARRIER_HALF then
                carrier_cnt <= carrier_cnt + 1;
            else
                carrier_cnt <= 0;
                carrier_38k <= not carrier_38k;
            end if;
        end if;
    end process;

    -- =============================================================
    -- 2. 发送逻辑 (TX)
    -- =============================================================
    process(clk, rst_n)
    begin
        if rst_n = '0' then
            tx_state <= IDLE;
            tx_cnt <= 0;
            ir_tx_pin <= '0';
        elsif rising_edge(clk) then
            case tx_state is
                when IDLE =>
                    ir_tx_pin <= '0';
                    if tx_trigger = '1' then
                        tx_state <= SEND_9MS;
                        tx_cnt <= 0;
                    end if;
                    
                when SEND_9MS =>
                    -- 输出 38kHz 载波
                    ir_tx_pin <= carrier_38k;
                    if tx_cnt < TX_TIME_9MS then
                        tx_cnt <= tx_cnt + 1;
                    else
                        tx_state <= SEND_GAP;
                        tx_cnt <= 0;
                    end if;
                    
                when SEND_GAP =>
                    -- 输出低电平 (不发光)
                    ir_tx_pin <= '0'; 
                    if tx_cnt < TX_TIME_4MS5 then
                        tx_cnt <= tx_cnt + 1;
                    else
                        tx_state <= DONE; -- 发送完毕
                    end if;
                    
                when DONE =>
                    ir_tx_pin <= '0';
                    tx_state <= IDLE; -- 回到空闲
            end case;
        end if;
    end process;

-- =============================================================
    -- 3. 接收逻辑 (RX) - 极简抗干扰版
    -- =============================================================
    process(clk, rst_n)
    begin
        if rst_n = '0' then
            rx_cnt <= 0;
            rx_detect <= '0';
        elsif rising_edge(clk) then
            
            -- VS1838B 特性：收到 38kHz 信号时输出【低电平 '0'】
            
            if ir_rx_pin = '0' then
                -- 收到信号了，开始累计时间
                if rx_cnt < 2000000 then -- 防止溢出
                    rx_cnt <= rx_cnt + 1;
                end if;
                
                -- 阈值判断：
                -- 只要低电平持续超过 2ms (100,000 个时钟周期)
                -- 就认为这是有效的启动信号！
                -- (不需要 9ms 那么长，2ms 足够区别于普通干扰了)
                if rx_cnt = 100000 then 
                    rx_detect <= '1'; -- 触发启动！
                else
                    rx_detect <= '0';
                end if;
                
            else
                -- 信号断了/没信号 (高电平)
                -- 立刻清零计数器，等待下一次
                rx_cnt <= 0;
                rx_detect <= '0';
            end if;
            
        end if;
    end process;
end Behavioral;