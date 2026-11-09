library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tracking_logic is
    Port ( 
        clk        : in  STD_LOGIC;
        rst_n      : in  STD_LOGIC;
        sensor_in  : in  STD_LOGIC_VECTOR(7 downto 0); 
        stop_req   : in  STD_LOGIC;   
        start_btn  : in  STD_LOGIC;   
        distance_mm: in INTEGER range 0 to 5000;
        
        -- 【新增】红外控制接口
        ir_start_in : in  STD_LOGIC; -- 来自红外接收模块
        tx_trigger  : out STD_LOGIC; -- 输出给红外发送模块
        
        speed_l    : out INTEGER range 0 to 100;
        speed_r    : out INTEGER range 0 to 100;
        debug_lap  : out STD_LOGIC_VECTOR(1 downto 0) 
    );
end tracking_logic;

architecture Behavioral of tracking_logic is

    -- === 基础速度参数 ===
    constant BASE_SPD : integer := 55; 
    constant TURN_HI  : integer := 60; 
    constant TURN_LO  : integer := 15; 
    constant ZERO     : integer := 0;
    
    -- === 跟随系统参数 ===
    constant FOLLOW_ACC_DIST   : integer := 200;  
    constant FOLLOW_DEC_DIST   : integer := 100;  
    constant FOLLOW_STOP_DIST  : integer := 60;   
    constant FOLLOW_TRACK_DIST : integer := 400;  
    constant ACC_RATIO         : integer := 130;  
    constant DEC_RATIO         : integer := 50;   
    constant NORMAL_RATIO      : integer := 100;  
    
    -- === 圈数与计时信号 ===
    signal state_lap : integer range 0 to 4 := 0; 
    signal last_act : integer range 0 to 2 := 0;
    constant COOLDOWN_MAX : integer := 50000000; 
    signal cooldown_timer : integer := 0;
    signal btn_deb_cnt : integer := 0;
    signal eff_sensor : STD_LOGIC_VECTOR(7 downto 0);
    constant BLACK_FILTER_TIME : integer := 750000; 
    signal black_duration_cnt : integer := 0;
    signal lap_count_flag : STD_LOGIC := '0'; 
    
    -- === Y字路口逻辑 ===
    signal y_junction_lock : STD_LOGIC := '0';
    signal y_lock_cnt      : integer range 0 to 5000000 := 0; 
    constant Y_JUNC_TRIG : STD_LOGIC_VECTOR(7 downto 0) := "00011000"; 

begin
    
    debug_lap <= std_logic_vector(to_unsigned(state_lap, 2)) when state_lap < 4 else "11";

    process(clk, rst_n)
        variable speed_l_tmp : integer range 0 to 100;
        variable speed_r_tmp : integer range 0 to 100;
        variable track_speed_l : integer range 0 to 100;
        variable track_speed_r : integer range 0 to 100;
        variable follow_ratio  : integer range 0 to 130 := 100;
    begin
        if rst_n = '0' then
            state_lap <= 0;
            speed_l <= 0;
            speed_r <= 0;
            last_act <= 0;
            cooldown_timer <= 0;
            black_duration_cnt <= 0;
            btn_deb_cnt <= 0;
            eff_sensor <= (others => '0');
            y_junction_lock <= '0';
            y_lock_cnt <= 0;
            lap_count_flag <= '0';
            tx_trigger <= '0'; -- 【新增】复位触发信号
            
        elsif rising_edge(clk) then
            
            -- =========================================================
            -- 状态0：等待启动 (修改部分)
            -- =========================================================
            if state_lap = 0 then
                speed_l <= 0;
                speed_r <= 0;
                y_junction_lock <= '0';
                y_lock_cnt <= 0;
                lap_count_flag <= '0';
                tx_trigger <= '0'; -- 默认不发送
                
                -- 【新增逻辑 A】红外启动通道 (跟随车用)
                -- 只要收到信号，跳过防抖，立刻启动
                if ir_start_in = '1' then
                    state_lap <= 1;
                    cooldown_timer <= COOLDOWN_MAX;
                    -- 跟随车被唤醒后，不需要发送信号给别人，所以保持 tx_trigger='0'
                
                -- 【新增逻辑 B】按键启动通道 (领头车用)
                elsif start_btn = '0' then
                    if btn_deb_cnt < 1000000 then
                        btn_deb_cnt <= btn_deb_cnt + 1;
                    else
                        state_lap <= 1;
                        cooldown_timer <= COOLDOWN_MAX;
                        btn_deb_cnt <= 0;
                        
                        -- 【重点】只有按键启动时，才触发红外发送，通知后车
                        tx_trigger <= '1'; 
                    end if;
                else
                    btn_deb_cnt <= 0;
                end if;

            -- =========================================================
            -- 状态4：跑完停车
            -- =========================================================
            elsif state_lap >= 4 then
                speed_l <= 0; 
                speed_r <= 0;
                y_junction_lock <= '0';
                y_lock_cnt <= 0;
                tx_trigger <= '0'; -- 停止发送

            -- =========================================================
            -- 状态1-3：行驶中
            -- =========================================================
            else 
                tx_trigger <= '0'; -- 【新增】运行中停止发送红外
                
                speed_l_tmp := 0;
                speed_r_tmp := 0;
                track_speed_l := 0;
                track_speed_r := 0;
                
                -- === 圈数计数逻辑 ===
                if cooldown_timer > 0 then
                    cooldown_timer <= cooldown_timer - 1;
                    black_duration_cnt <= 0;
                    lap_count_flag <= '0';
                else
                    if (sensor_in = "11111111" or sensor_in = "01111110") then
                        if black_duration_cnt < BLACK_FILTER_TIME then
                            black_duration_cnt <= black_duration_cnt + 1;
                        else
                            if lap_count_flag = '0' and state_lap < 3 then
                                state_lap <= state_lap + 1;
                                lap_count_flag <= '1';
                            elsif state_lap = 3 then
                                state_lap <= 4; 
                                lap_count_flag <= '1';
                            end if;
                            cooldown_timer <= COOLDOWN_MAX;
                        end if;
                    else
                        black_duration_cnt <= 0;
                    end if;
                end if;

                -- === Y字路口锁定逻辑 ===
                eff_sensor <= sensor_in;
                if sensor_in = Y_JUNC_TRIG then 
                    y_junction_lock <= '1';
                    y_lock_cnt <= 5000000;
                elsif y_lock_cnt > 0 then
                    y_junction_lock <= '1';
                    y_lock_cnt <= y_lock_cnt - 1;
                else
                    y_junction_lock <= '0';
                end if;
        
                -- === 循迹速度计算 ===
                case eff_sensor is
                    when "00011000" | "00010000" | "00001000" | "00111100" =>
                        track_speed_l := BASE_SPD; track_speed_r := BASE_SPD; last_act <= 0;
                    when "00100000" | "00110000" | "01100000" | "10000000" | "11000000" | "11100000" => 
                        track_speed_l := TURN_HI; track_speed_r := TURN_LO; last_act <= 1;
                    when "00000100" | "00000110" | "00001100" | "00000010" | "00000001" | "00000111" =>
                        track_speed_l := TURN_LO; track_speed_r := TURN_HI; last_act <= 2;
                    when "11111111" | "01111110" =>
                        track_speed_l := BASE_SPD; track_speed_r := BASE_SPD;
                    when "00000000" =>
                        if last_act = 1 then     
                            track_speed_l := TURN_HI; track_speed_r := ZERO; 
                        elsif last_act = 2 then  
                            track_speed_l := ZERO; track_speed_r := TURN_HI;
                        else                     
                            track_speed_l := BASE_SPD; track_speed_r := BASE_SPD;
                        end if;
                    when others =>
                        track_speed_l := BASE_SPD; track_speed_r := BASE_SPD;
                end case;
                
                -- === 跟随速度调整 ===
                if distance_mm = 0 or distance_mm > FOLLOW_TRACK_DIST then
                    follow_ratio := NORMAL_RATIO;
                elsif distance_mm < FOLLOW_STOP_DIST then
                    follow_ratio := 0;
                elsif distance_mm < FOLLOW_DEC_DIST then
                    follow_ratio := DEC_RATIO;
                elsif distance_mm > FOLLOW_ACC_DIST then
                    follow_ratio := ACC_RATIO;
                else
                    follow_ratio := NORMAL_RATIO;
                end if;
                
                -- === 优先级逻辑 ===
                if y_junction_lock = '1' then
                    speed_l_tmp := track_speed_l;
                    speed_r_tmp := track_speed_r;
                else
                    if follow_ratio = 0 then
                        speed_l_tmp := 0;
                        speed_r_tmp := 0;
                    else
                        speed_l_tmp := (track_speed_l * follow_ratio) / 100;
                        speed_r_tmp := (track_speed_r * follow_ratio) / 100;
                        if speed_l_tmp > 100 then speed_l_tmp := 100; end if;
                        if speed_r_tmp > 100 then speed_r_tmp := 100; end if;
                    end if;
                end if;
                
                -- === 第二圈强制左转 ===
                if state_lap = 2 and cooldown_timer < 40000000 then
                    if (sensor_in(1) = '1') and (sensor_in(7)='0' and sensor_in(6)='0')then
                        speed_r_tmp := 80; speed_l_tmp := 10; last_act <= 1;
                    end if;
                end if;
                
                -- 输出最终速度
                speed_l <= speed_l_tmp;
                speed_r <= speed_r_tmp;
                
            end if; -- end state check
            
        end if; -- end rising_edge
    end process;

end Behavioral;