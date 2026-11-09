library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity smart_car is
    Port ( 
        sys_clk   : in  STD_LOGIC; -- 50MHz 时钟
        sys_rst_n : in  STD_LOGIC; -- 复位按键
        key_start : in STD_LOGIC; -- 启动按键
        
        -- 1. 8路灰度模块接口
        sensor_out : in  STD_LOGIC;
        sensor_ad  : out STD_LOGIC_VECTOR(2 downto 0);
        
        -- 2. 超声波接口
        echo : in  STD_LOGIC;
        trig : out STD_LOGIC;
        
        -- 3. 电机驱动接口 (TB6612 / L298N)
        -- 左电机
        mot_l_pwm : out STD_LOGIC; 
        mot_l_in1 : out STD_LOGIC; 
        mot_l_in2 : out STD_LOGIC;
        
        -- 右电机
        mot_r_pwm : out STD_LOGIC; 
        mot_r_in1 : out STD_LOGIC; 
        mot_r_in2 : out STD_LOGIC;
        
        -- 4. 调试与指示 LED
        led_debug : out STD_LOGIC; -- 障碍物停车指示
        led_lap   : out STD_LOGIC_VECTOR(1 downto 0); -- 圈数指示
        
        -- 蜂鸣器输出端口（CE4EP6F17C8开发板）
        buzzer    : out STD_LOGIC;  
		  
		  		  -- 红外物理接口
		  ir_tx : out STD_LOGIC; -- 接发射管
		  ir_rx : in  STD_LOGIC  -- 接接收头
    );
end smart_car;

architecture Behavioral of smart_car is

    -- === 组件声明区 ===
    component sensor_scanner is
        Generic ( STABLE_TIME : integer := 5000 ); 
        Port ( 
            clk, rst_n : in STD_LOGIC;
            sensor_out : in STD_LOGIC;
            ad_pins    : out STD_LOGIC_VECTOR(2 downto 0);
            data_ready : out STD_LOGIC_VECTOR(7 downto 0)
        );
    end component;

    component tracking_logic is
        Port ( 
            clk        : in STD_LOGIC;
            rst_n      : in STD_LOGIC;
            sensor_in  : in STD_LOGIC_VECTOR(7 downto 0);
            stop_req   : in STD_LOGIC;   -- 超声波停车信号
            start_btn  : in STD_LOGIC;   -- 启动按键
            distance_mm: in INTEGER range 0 to 5000; -- 距离输入(mm)
            speed_l    : out INTEGER range 0 to 100;
            speed_r    : out INTEGER range 0 to 100;
            debug_lap  : out STD_LOGIC_VECTOR(1 downto 0);
				-- 红外物理接口
            ir_start_in : in STD_LOGIC;
            tx_trigger  : out STD_LOGIC
		   );
		end component;

    component hc_sr04 is
        Port ( 
            clk         : in STD_LOGIC;
            rst_n       : in STD_LOGIC;
            echo        : in STD_LOGIC;
            trig        : out STD_LOGIC;
            stop_flag   : out STD_LOGIC;  -- 障碍物停车信号
            distance_mm : out INTEGER range 0 to 5000 -- 距离输出(mm)
        );
    end component;

    component motor_pwm is
        Port ( 
            clk, rst_n : in STD_LOGIC;
            duty       : in INTEGER range 0 to 100;
            pwm_out    : out STD_LOGIC
        );
    end component;
	 
	 -- 红外组件 (新增)
    component ir_sync is
        Port ( 
            clk        : in  STD_LOGIC;
            rst_n      : in  STD_LOGIC;
            tx_trigger : in  STD_LOGIC;
            ir_tx_pin  : out STD_LOGIC;
            ir_rx_pin  : in  STD_LOGIC;
            rx_detect  : out STD_LOGIC
        );
    end component;

    -- === 内部信号连线区 ===
    signal w_sensor_data : STD_LOGIC_VECTOR(7 downto 0);
    signal w_stop_flag   : STD_LOGIC;
    signal w_speed_l     : INTEGER range 0 to 100;
    signal w_speed_r     : INTEGER range 0 to 5000;
    signal w_lap_info    : STD_LOGIC_VECTOR(1 downto 0);
    signal w_distance_mm : INTEGER range 0 to 5000;
	 -- 红外握手信号
    signal w_ir_tx_trig : STD_LOGIC; -- 逻辑 -> 发送
    signal w_ir_rx_det  : STD_LOGIC; -- 接收 -> 逻辑
    
    -- === 蜂鸣器专用控制信号 ===
    signal buzzer_en      : STD_LOGIC := '0'; -- 蜂鸣器使能
    signal buzzer_has_ring: STD_LOGIC := '0'; -- 防重复触发标志
    signal buzzer_cnt     : INTEGER range 0 to 25000; -- 1kHz方波计数
    signal buzzer_pwm     : STD_LOGIC := '0'; -- 蜂鸣器方波输出
    signal buzzer_dur_cnt : INTEGER range 0 to 100000000; -- 发声时长计数
    constant BUZZER_RING_DUR : INTEGER := 50000000; -- 发声1秒（可调整）

begin

    -- ==================================================
    -- 蜂鸣器核心控制逻辑：仅停车（w_lap_info="11"+电机停转）时响一次
    -- ==================================================
    process(sys_clk, sys_rst_n)
    begin
        if sys_rst_n = '0' then
            buzzer_en <= '0';
            buzzer_has_ring <= '0';
            buzzer_dur_cnt <= 0;
        elsif rising_edge(sys_clk) then
            -- 触发条件：1. 圈数标识=11 2. 左电机速度=0（停车） 3. 未响过
            if w_lap_info = "11" and w_speed_l = 0 and buzzer_has_ring = '0' then
                buzzer_en <= '1';
                if buzzer_dur_cnt < BUZZER_RING_DUR then
                    buzzer_dur_cnt <= buzzer_dur_cnt + 1;
                else
                    buzzer_en <= '0';
                    buzzer_has_ring <= '1'; -- 标记已响，不再重复
                    buzzer_dur_cnt <= 0;
                end if;
            else
                buzzer_en <= '0';
            end if;
        end if;
    end process;

    -- ==================================================
    -- 1kHz方波生成（驱动蜂鸣器发声）
    -- ==================================================
    process(sys_clk, sys_rst_n)
    begin
        if sys_rst_n = '0' then
            buzzer_cnt <= 0;
            buzzer_pwm <= '0';
        elsif rising_edge(sys_clk) then
            if buzzer_en = '1' then
                if buzzer_cnt >= 25000 then -- 50MHz/1kHz/2 = 25000
                    buzzer_cnt <= 0;
                    buzzer_pwm <= not buzzer_pwm;
                else
                    buzzer_cnt <= buzzer_cnt + 1;
                end if;
            else
                buzzer_cnt <= 0;
                buzzer_pwm <= '0'; -- 未使能时关闭
            end if;
        end if;
    end process;

    -- 蜂鸣器输出赋值
    buzzer <= buzzer_pwm;

    -- ==================================================
    -- 原有逻辑
    -- ==================================================
    -- 1. 实例化传感器扫描
    u_scan: sensor_scanner
    port map(
        clk => sys_clk, 
        rst_n => sys_rst_n,
        sensor_out => sensor_out, 
        ad_pins => sensor_ad,
        data_ready => w_sensor_data
    );

    -- 2. 实例化超声波
    u_sonic: hc_sr04
    port map(
        clk => sys_clk, 
        rst_n => sys_rst_n,
        echo => echo, 
        trig => trig,
        stop_flag => w_stop_flag,
        distance_mm => w_distance_mm
    );
	 
	 -- 2. 实例化红外模块 
    u_ir: ir_sync
    port map(
        clk => sys_clk,
        rst_n => sys_rst_n,
        -- 连接物理引脚 (确保你在 Entity Port 里定义了 ir_tx 和 ir_rx)
        ir_tx_pin => ir_tx, 
        ir_rx_pin => ir_rx,
        -- 连接逻辑信号
        tx_trigger => w_ir_tx_trig,
        rx_detect  => w_ir_rx_det
    );
    
    led_debug <= w_stop_flag; 

    -- 3. 实例化控制逻辑
    u_logic: tracking_logic
    port map(
        clk => sys_clk, 
        rst_n => sys_rst_n,
        sensor_in => w_sensor_data, 
        stop_req => w_stop_flag,
        start_btn => key_start,
        distance_mm => w_distance_mm,
        speed_l => w_speed_l, 
        speed_r => w_speed_r,
        debug_lap => w_lap_info,
		  ir_start_in => w_ir_rx_det, -- 连入
		  tx_trigger  => w_ir_tx_trig -- 连出
    );

    led_lap <= w_lap_info; 

    -- 4. 实例化电机PWM输出
    u_pwm_l: motor_pwm
    port map( 
        clk => sys_clk, 
        rst_n => sys_rst_n, 
        duty => w_speed_l, 
        pwm_out => mot_l_pwm 
    );
    
    u_pwm_r: motor_pwm
    port map( 
        clk => sys_clk, 
        rst_n => sys_rst_n, 
        duty => w_speed_r, 
        pwm_out => mot_r_pwm 
    );

    -- 5. 电机方向控制
    mot_l_in1 <= '1'; 
    mot_l_in2 <= '0';
    mot_r_in1 <= '1'; 
    mot_r_in2 <= '0';

end Behavioral;