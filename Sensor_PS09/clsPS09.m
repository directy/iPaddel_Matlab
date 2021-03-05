classdef clsPS09< handle
    %UNTITLED Summary of this class goes here
    %   Website: https://www.pmt-fl.com/picostrain/picostrain-ps09
    
    properties
        
    i2cobj
        
    power_reset =       0xF0;  % Registerreset durchführen 
    init_reset =        0xC0;
    start_newcycle =    0xCC;
    start_TDC_cycle =   0xCE;
    watchdog_off =      0x9E; % Ausgeschalten, da Reset im Fehlerfall
    watchdog_on =      0x9F
    
    % main instructions
    i2cWrite = 0x00;
    i2cRead = 0x40;
        
    % Setup -Registerzuweisung
    
    c_reg0 = 0x30;
    c_reg1 = 0x31;  
    c_reg2 = 0x32; 
    c_reg3 = 0x33;
    c_reg4 = 0x34;
    c_reg5 = 0x35;
    c_reg6 = 0x36;
    c_reg7 = 0x37; 
    c_reg8 = 0x38;  
    c_reg9 = 0x39; 
    c_reg10 = 0x3A; 
    c_reg11 = 0x3B; 
    c_reg12 = 0x3C; 
    c_reg13 = 0x3D; 
    c_reg14 = 0x3E; 
    c_reg15 = 0x3F; 

    ValueCreg_0;
    ValueCreg_1;
    ValueCreg_2;
    ValueCreg_3;
    ValueCreg_4;
    ValueCreg_5;
    ValueCreg_6;
    ValueCreg_7;
    ValueCreg_8;
    ValueCreg_9;
    ValueCreg_10;
    ValueCreg_11;
    ValueCreg_12;
    ValueCreg_13;
    ValueCreg_14;
    ValueCreg_15;
   
    %% Parameter for Register
     
            % c_reg0 = 0x30;
            % --------------
        
            ConfigParam_00 = struct(...
                 'connect_vcc_rosc', [ 00 0   1  0 ],... %
                 'cpu_speed',        [ 00 1   2  1 ],... %
                 'mod_math',         [ 00 3   1  0 ],... %
                 'osz10khz_fsoc',    [ 00 4   4  13],... %
                 'io_a',             [ 00 8   8  -1],... %
                 'tdc_conv_cnt',     [ 00 16  8  -1]... %
                 )
         
            %c_reg1 = 0x31;
            % --------------
            ConfigParam_01 = struct(...
                'otp_usr_prg',          [ 01 0   1  -1],... %
                'otp_pwr_prg',          [ 01 1   1  -1],... %
                'otp_pwr_cfg',          [ 01 2   1  -1],... %
                'dis_wheat_pp',         [ 01 3   1  0 ],... %
                'single_conv_extern',   [ 01 04  2  -1],... %
                'mod_rspan',            [ 01 6   1  -1],... %
                'en_gain',              [ 01 7   1  1 ],... %
                'integrated_rspan',     [ 01 8   1  -1],... %
                'en_avcal',             [ 01 9   1  0 ],... %
                'mult_en_ub',           [ 01 10  1  1 ],... %
                'en_TkPar',             [ 01 11  1  0 ],... %
                'sel_compth2',          [ 01 12  1  0 ],... %
                'sel_comp_r',           [ 01 13  2  2 ],... %
                'ps_noise_en',          [ 01 15  1  0 ],... %
                'ps_shift_clk_noise',   [ 01 16  1  0 ],... %
                'tdc_sleepmode',        [ 01 17  1  -1],... %
                'mr2_en',               [ 01 18  1  1 ],... %
                'adj_hr',               [ 01 19  4  5 ],... % Config Register StartBit Anzahl Bits defaultValue
                'en_emi_meas_gain_comp',[ 01 23  1  1 ]... %
                 )
             
            % c_reg2 = 0x32;
            % --------------
            
            ConfigParam_02 = struct(...
                 'port_pat',         [ 02 0   2   0 ],...%
                 'single_conversion',[ 02 2   1  -1 ],...%
                 'auto10k',          [ 02 3   1   0 ],...%
                 'cytime',           [ 02 4   10 -1 ],...%
                 'avrate',           [ 02 14  10 -1 ]... %
                 )

    
            % c_reg3 = 0x33;
            % --------------
            
            ConfigParam_03 = struct(...
                'bridge',            [ 03 0   2 -1],... %   
                'mfake',             [ 03 2   2 2 ],... %
                'ps_tdc1_adjust',    [ 03 4   6 23],... % fix
                'en_sdel_noise',     [ 03 10  1 0 ],... % fix
                'ps_dis',            [ 03 11  1 0 ],... % fix
                'stretch',           [ 03 12  2 -1],... %
                'dis_noise4',        [ 03 14  1 0 ],... % fix
                'neg_sense',         [ 03 15  1 0 ],... % fix
                'force_quattro_mode',[ 03 16 1 0  ],... % fix
                'sel_start_osz',     [ 03 17  3 3 ],... %
                'uart_en',           [ 03 20  1 -1],... %
                'en_wheatstone',     [ 03 21  1 -1],... %
                'sel_startdel',      [ 03 22  2 2 ]... %
                 )
             
            % c_reg4 = 0x34;
            % --------------
                
            ConfigParam_04 = struct(...
                'mult_Hb1',         [ 04 00  24 1]... %
                )
            
            % c_reg5 = 0x35;
            % --------------
            
            ConfigParam_05 = struct(...              
                'mult_Hb2',         [ 05 00  24 1]... %
                )
                 
            % c_reg6 = 0x36;
            % --------------
            
            ConfigParam_06 = struct(...
                'mult_Hb3',         [ 06 00  24 1]... %
                )
             
            % c_reg7 = 0x37;
            % --------------
            
            ConfigParam_07 = struct(...
                'mult_Hb4',         [ 07 00  24 1 ],... %
                'mult_TkPar',       [ 07 00  24 -1] ... %
                )
             
            % c_reg8 = 0x38;
            % --------------
          
            ConfigParam_08 = struct(...
                'Tk_Gain',         [ 08 0  24 0x100000]... %
                )
             
            % c_reg9 = 0x39;
            % --------------
            
            ConfigParam_09 = struct(...
                'Tk_Offset',          [ 09 0  24 0]... %
                )
         
            % c_reg10 = 0x3A;
            % --------------
             
            ConfigParam_10 = struct(...
                'gain_comp',        [ 10 0   8 164 ], ... %
                'Mult_Ub',          [ 10 8   8 0xFB],... %
                'calcor',           [ 10 16  8 0   ]... %
                )
         
            % c_reg10 = 0x3B;
            % --------------
             
            ConfigParam_11 = struct(...
                'io_en_0_sdo',      [ 11 00  2 -1],... %
                'io_en_1_sdi',      [ 11 02  2 -1],... %
                'io_en_2_sck',      [ 11 04  2 -1],... %
                'io_en_3_mio',      [ 11 06  2 -1],... %
                'io_en_4_mio',      [ 11 08  2 -1],... %
                'io_en_5_mio',      [ 11 10  2 -1],... %
                'io_en_6_mio',      [ 11 12  2 -1],... %
                'io_en_7_mio',      [ 11 14  2 -1],... %
                'io_en_digital',    [ 11 16  8 1]  ... %
                )
             
            % c_reg10 = 0x3C;
            % --------------
         
            ConfigParam_12 = struct(...
                 'acam_internal_bit1',    [ 12 0   2  0],... %
                 'usr_epr_prg_time',      [ 12 2   2  0], ... % 
                 'acam_internal_bit2',    [ 12 4   1  0],... %  
                 'usr_epr_always_on',     [ 12 5   1  0],... %
                 'lcd_clk_open_drain',    [ 12 6   1  0],... %
                 'en_emi_noise_reduction',[ 12 7   1  1],... %
                 'sel_refresh_vlt',       [ 12 8   2  2],... %
                 'lcd_clk_sel',           [ 12 10  2  3],... %
                 'multio3_sel',           [ 12 12  2 -1],... %
                 'irq_dsp_en',            [ 12 14  1 -1],... %
                 'irq_dsp_edge',          [ 12 15  1 0 ],... %
                 'ext_eeprom_clk_speed',  [ 12 16  2 1 ],... %
                 'con_comp',              [ 12 18  2 1 ],... %
                 'sel_comp_int',          [ 12 20  1 -1],... %
                 'dis_pp_cycle_mod',      [ 12 21  1 1 ],... %
                 'lcd_clk_pol',           [ 12 22  1 0 ],... %
                 'irq_dsp_pin_sel',       [ 12 23  1 0 ] ... %
                 )
             
            % c_reg10 = 0x3D;
            % --------------
         
            ConfigParam_13 = struct(...
                 'multio4_sel',      [ 13 00  4  -1 ],... %
                 'crf_sen2',         [ 13 1   1  10 ],... %
                 'selqha',           [ 13 4   6  45 ],... %
                 'sense_discharge',  [ 13 10  1  1  ], ... %
                 'mi_enable',        [ 13 11  3 -1  ],... %
                 'mi_updaterate',    [ 13 14  2  1  ],... %
                 'mi_sel_clk5k',     [ 13 16  1  0  ],... %
                 'sel_rc_osc2',      [ 13 17  1 -1  ],... %
                 'crf_tau',          [ 13 18  1  0  ],... %
                 'sel_rc_osc1',      [ 13 19  1 -1  ],... %
                 'crf_tp1',          [ 13 20  1  0  ],... %
                 'crf_sen3',         [ 13 22  1  0  ],... %
                 'store_tdc_times',  [ 13 23  1  0  ]... %
                 )
             
            % c_reg10 = 0x3E;
            % --------------
         
            ConfigParam_14 = struct(...
                'sel_rtemp_300R',   [ 14 0   1 -1 ],... %
                'caltime',          [ 14 1   2 1  ],... %
                'alupernopen',      [ 14 3   1 3  ],... %
                'internalBits',     [ 14 5   4 1  ],... %
                'acam_internal',    [ 14 8   4 1  ],... %
                'upd_vlt',          [ 14 12  3 0  ], ... %
                'en_pp_measurement',[ 14 15  1 1  ],... %
                'iternal_1',        [ 14 15  3 0  ],... %
                'bandgap_trim',     [ 14 19  4 10 ],... % 
                'iternal_2',        [ 14 23  1 0  ]... %
                )
             
            % c_reg10 = 0x3F;
            % --------------
         
            ConfigParam_15 = struct(...
                 'cport_adapt_speed',  [ 15 0   8 1 ],... %
                 'cport_thresh2',      [ 15 8   8 -1 ],... %
                 'cport_en',           [ 15 16  4 0  ],... %
                 'cport_r',            [ 15 20  2 0  ],... %
                 'cport_update',       [ 15 22  1 0  ], ... %
                 'cport_adapt',        [ 15 23  1 1  ]... %
                 )
             
         %%
          ConfigParam_80 = struct(...
             'uart_rec_buf',             [ 80 0   8 -1 ],... %
             'scon',                     [ 80 8   3 -1 ],... %
             'status_uart_rx_data_par',  [ 80 11  1 -1 ],...  % status only (not to write)
             'intern',                   [ 80 12  10 0 ],...  %
             'status_uart_start',        [ 80 22  1 -1 ],...  % status only (not to write)
             'status_uart_stop',         [ 80 23  1 -1 ] ...  % status only (not to write)    
             )
             
          ConfigParam_84 = struct(...
             'intern',              [ 84 0  16 -1],... %
             'uart_sbuf_i15',       [ 84 16  8 -1]... %
             )
             
          ConfigParam_86 = struct(...
             'uart_sbuf_i0',        [ 86 0   8 -1 ],... %       
             'uart_sbuf_i1',        [ 86 8   8 -1 ],... %
             'uart_sbuf_i2',        [ 86 16  8 -1 ] ... %
             )
             
          ConfigParam_87 = struct(...
             'uart_sbuf_i3',        [ 87 0   8 -1 ],... %
             'uart_sbuf_i4',        [ 87 8   8 -1 ],... %
             'uart_sbuf_i5',        [ 87 16  8 -1 ] ... %
             )
             
          ConfigParam_88 = struct(...
             'uart_sbuf_i6',        [ 88 0   8 -1],... %
             'uart_sbuf_i7',        [ 88 8   8 -1],... %
             'uart_sbuf_i8',        [ 88 16  8 -1] ... %
             )
             
          ConfigParam_89 = struct(...
             'uart_sbuf_i10',       [ 89 8   8 -1  ],... %
             'uart_sbuf_i11',       [ 89 16  8 -1  ],... %
             'uart_sbuf_i9',        [ 89 0   8 1 -1] ... %
             )
             
          ConfigParam_90 = struct(...
             'uart_sbuf_i12',       [ 90 0   8 -1],... %
             'uart_sbuf_i13',       [ 90 8   8 -1],... %
             'uart_sbuf_i14',       [ 90 16  8 -1] ... %
             )
             
          ConfigParam_91 = struct(...            
             'uart_tx_cnt',         [ 91 0   4 20 ],... %
             'uart_baud_rate',      [ 91 4   4 5 ],... %
             'uart_rdx_sel',        [ 91 8   2 0 ], ... %
             'uart_trans',          [ 91 10  1 -1],... %
             'uart_par',            [ 91 11  1 -1],... %
             'uart_rec_en',         [ 91 12  1 -1],... %
             'uart_mode',           [ 91 13  1 -1],... %
             'uart_rec_int_dis',    [ 91 14  1 0 ],... %
             'uart_auto_det_stop',  [ 91 15  1 -1],... %
             'uart_mpcomm',         [ 91 16  1 -1],... %
             'uart_rec_int_ack',    [ 91 17  1 -1],... %
             'uart_4Mhz_divider',   [ 91 18  1 1 ],... %
             'uart_clk_en',         [ 91 19  1 0 ],... %
             'uart_tx_cnt_MSB',     [ 91 20  3 0 ],... %
             'irq_uart_en',         [ 91 23  1 -1]... %   
            )
        
% 1) mi = multi-input
% 2) cport = capacitive ports
% 3) ps = phase shifter
% 4) mpcomm = multi-processor communication
% 5) scon = serial port control register
% 6) par = parity        
        
        
    end
     
    methods
        function obj = AdvancedInit(obj)
           %INIT Construct an instance of this class
           % Initalisiert den PS09
           % schreibe auf Register | Adresse im RAM | 3 x 8 Bytes
           % c_reg0 = [ hex2dec('00'), hex2dec('30'), hex2dec('23'), hex2dec('00'), hex2dec('D2')];
            
           % value_reg0 = [obj.i2cWrite , obj.c_reg0, hex2dec('23'), hex2dec('00'), hex2dec('D2')];
            
           % ConfigParam_00 = struct(...
           %      'connect_vcc_rosc', [ 00 0   1  0 ],... %
           %      'cpu_speed',        [ 00 1   2  1 ],... %
           %      'mod_math',         [ 00 3   1  0 ],... %
           %      'osz10khz_fsoc',    [ 00 4   4  13],... %
           %      'io_a',             [ 00 8   8  -1],... %
           %      'tdc_conv_cnt',     [ 00 16  8  -1]... %
           %      )
               
            value1 = setParam(obj,obj.ConfigParam_00.connect_vcc_rosc,0);
            value2 = setParam(obj,obj.ConfigParam_00.cpu_speed,1);
            value3 = setParam(obj,obj.ConfigParam_00.mod_math,0);
            value4 = setParam(obj,obj.ConfigParam_00.osz10khz_fsoc,13); % S. 5-4 PS09-Doku
            value5 = setParam(obj,obj.ConfigParam_00.io_a,0);
            value6 = setParam(obj,obj.ConfigParam_00.tdc_conv_cnt,0x23);
            
            creg_0 = bitconcat( value6, value5, value4,...
                                value3, value2, value1);
            
            %split Value in 3 Bytes
            
            ValueCreg_0 = bin2dec(creg_0.bin(1:8));
            ValueCreg_0 = [ValueCreg_0 bin2dec(creg_0.bin(9:16))];
            ValueCreg_0 = [ValueCreg_0 bin2dec(creg_0.bin(17:24))];
            
            % change Property in Object
            obj.ValueCreg_0 = ValueCreg_0;
           
            status = [obj.i2cWrite , obj.c_reg0, ValueCreg_0(1),ValueCreg_0(2), ValueCreg_0(3)];
            
            
           % ConfigParam_01 = struct(...
           %    'adj_hr',               [ 01 19  4  5 ],... % Config Register StartBit Anzahl Bits defaultValue
           %    'dis_wheat_pp',         [ 01 3   1  0 ],... %
           %    'en_avcal',             [ 01 9   1  0 ],... %
           %    'en_emi_meas_gain_comp',[ 01 23  1  1 ],... %
           %    'en_TkPar',             [ 01 11  1  0 ],... %
           %    'en_gain',              [ 01 7   1  1 ],... %
           %    'mod_rspan',            [ 01 6   1  -1],... %
           %    'mr2_en',               [ 01 18  1  1 ],... %
           %    'mult_en_ub',           [ 01 10  1  1 ],... %
           %    'integrated_rspan',     [ 01 8   1  -1],... %
           %    'otp_pwr_cfg',          [ 01 2   1  -1],... %
           %    'otp_pwr_prg',          [ 01 1   1  -1],... %
           %    'otp_usr_prg',          [ 01 0   1  -1],... %
           %    'ps_noise_en',          [ 01 15  1  0 ],... %
           %    'ps_shift_clk_noise',   [ 01 16  1  0 ],... %
           %    'sel_comp_r',           [ 01 13  2  2 ],... %
           %    'sel_compth2',          [ 01 12  1  0 ],... %
           %    'tdc_sleepmode',        [ 01 17  1  -1],... %
           %    'single_conv_extern',   [ 01 04  2  -1]... %
           %   )

            value1 =  setParam(obj,obj.ConfigParam_01.otp_usr_prg  ,0);
            value2 =  setParam(obj,obj.ConfigParam_01.otp_pwr_prg  ,0);
            value3 =  setParam(obj,obj.ConfigParam_01.otp_pwr_cfg  ,0);
            value4 =  setParam(obj,obj.ConfigParam_01.dis_wheat_pp ,0);% fix
            value5 =  setParam(obj,obj.ConfigParam_01.single_conv_extern,0);
            value6 =  setParam(obj,obj.ConfigParam_01.mod_rspan    ,1);
            value7 =  setParam(obj,obj.ConfigParam_01.en_gain      ,1);
            value8 =  setParam(obj,obj.ConfigParam_01.integrated_rspan,1);
            value9 =  setParam(obj,obj.ConfigParam_01.en_avcal     ,0); % fix
            value10 = setParam(obj,obj.ConfigParam_01.mult_en_ub   ,1);
            value11 = setParam(obj,obj.ConfigParam_01.en_TkPar     ,0);
            value12 = setParam(obj,obj.ConfigParam_01.sel_compth2  ,0);
            value13 = setParam(obj,obj.ConfigParam_01.sel_comp_r   ,2);
            value14 = setParam(obj,obj.ConfigParam_01.ps_noise_en  ,0); % fix
            value15 = setParam(obj,obj.ConfigParam_01.ps_shift_clk_noise,0); % fix
            value16 = setParam(obj,obj.ConfigParam_01.tdc_sleepmode,0);
            value17 = setParam(obj,obj.ConfigParam_01.mr2_en       ,1);
            value18 = setParam(obj,obj.ConfigParam_01.adj_hr       ,5); %fix
            value19 = setParam(obj,obj.ConfigParam_01.en_emi_meas_gain_comp,1); % fix
    
            creg_1 = bitconcat(value19, value18, value17, ...
                               value16, value15, value14, ...
                               value13, value12, value11, ...
                               value10, value9,  value8,  ...
                               value7,  value6,  value5,  ...
                               value4,  value3,  value2,  ...
                               value1);
    
            ValueCreg_1 = bin2dec(creg_1.bin(1:8)); % Wert abweichend von Maik, da en_emi_meas_gain_comp = 1 nicht 0
            ValueCreg_1 = [ValueCreg_1 bin2dec(creg_1.bin(9:16))];
            ValueCreg_1 = [ValueCreg_1 bin2dec(creg_1.bin(17:24))];
    
            % change Property in Object
            obj.ValueCreg_1 = ValueCreg_1;
  
           % ConfigParam_02 = struct(...
           %   'auto10k',          [ 02 3   1    0],... % fix
           %   'single_conversion',[ 02 2   1   -1],... %
           %   'avrate',           [ 02 14  10  -1],... %
           %   'cytime',           [ 02 4   10  -1] ... %
           %   )
         
         
            value1 =  setParam(obj,obj.ConfigParam_02.port_pat  ,2);    % fix
            value2 =  setParam(obj,obj.ConfigParam_02.single_conversion  ,0);
            value3 =  setParam(obj,obj.ConfigParam_02.auto10k,1); % fix Abweichung von Maik auto 10K=0
            value4 =  setParam(obj,obj.ConfigParam_02.cytime  ,0x47);
            value5 =  setParam(obj,obj.ConfigParam_02.avrate  ,0x155);

            creg_2 = bitconcat( value5, value4, value3, value2, value1 );
      
            ValueCreg_2 = bin2dec(creg_2.bin(1:8)); 
            ValueCreg_2 = [ValueCreg_2 bin2dec(creg_2.bin(9:16))];
            ValueCreg_2 = [ValueCreg_2 bin2dec(creg_2.bin(17:24))];
            
            % change Property in Object
            obj.ValueCreg_2 = ValueCreg_2;

    
           % ConfigParam_03 = struct(...
           %    'dis_noise4',       [ 03 14  1 0],... % fix
           %    'bridge',           [ 03 0   2 -1 ],... %     
           %    'en_sdel_noise',    [ 03 10  1 0],... % fix
           %    'en_wheatstone',    [ 03 21  1 -1],... %
           %    'force_quattro_mode',[ 03 16 1 0],... % fix
           %    'mfake',            [ 03 2   2 2],... %
           %    'neg_sense',        [ 03 15  1 0],... % fix
           %    'ps_dis',           [ 03 11  1 0],... % fix
           %    'ps_tdc1_adjust',   [ 03 4   6 23],... % fix
           %    'sel_start_osz',    [ 03 17  3 3],... %
           %    'sel_startdel',     [ 03 22  2 2],... %
           %    'uart_en',          [ 03 20  1 -1],... %
           %    'stretch',          [ 03 12  2 -1]... %
           %    )
         
          value1 =   setParam(obj,obj.ConfigParam_03.bridge,0);   
          value2 =   setParam(obj,obj.ConfigParam_03.mfake,2);
          value3 =   setParam(obj,obj.ConfigParam_03.ps_tdc1_adjust,0x17);
          value4 =   setParam(obj,obj.ConfigParam_03.en_sdel_noise,0);
          value5 =   setParam(obj,obj.ConfigParam_03.ps_dis,0);
          value6 =   setParam(obj,obj.ConfigParam_03.stretch,0);
          value7 =   setParam(obj,obj.ConfigParam_03.dis_noise4,0);
          value8 =   setParam(obj,obj.ConfigParam_03.neg_sense,0);
          value9 =   setParam(obj,obj.ConfigParam_03.force_quattro_mode,0);
          value10 =  setParam(obj,obj.ConfigParam_03.sel_start_osz,1);
          value11 =  setParam(obj,obj.ConfigParam_03.uart_en,0);
          value12 =  setParam(obj,obj.ConfigParam_03.en_wheatstone,0);
          value13 =  setParam(obj,obj.ConfigParam_03.sel_startdel,2);
          
          creg_3 = bitconcat( value13, value12, value11, ...
                              value10, value9,  value8,  ...
                              value7,  value6,  value5,  ...
                              value4,  value3,  value2,  ...
                              value1);
                          
          ValueCreg_3 = bin2dec(creg_3.bin(1:8)); 
          ValueCreg_3 = [ValueCreg_3 bin2dec(creg_3.bin(9:16))];
          ValueCreg_3 = [ValueCreg_3 bin2dec(creg_3.bin(17:24))];
          
          % change Property in Object
          obj.ValueCreg_3 = ValueCreg_3;
          
%%
%              ConfigParam_04 = struct(...
%              'mult_Hb1',         [ 04 00  24 1]... %
%              )
% Multipliationsfaktor ermöglicht es bei Benutzung von zwei oder mehr
% Halbbrücken die Empfindlichkeit aufeinander anzupassen. Die Werte müssen
% dabei über Messungen mit Nennlast ermittelt werden.
% gewählte Einstellun 2^22 --> 0xF400000
          creg_4 =   setParam(obj,obj.ConfigParam_04.mult_Hb1,2^22); 
          
          ValueCreg_4 = bin2dec(creg_4.bin(1:8)); 
          ValueCreg_4 = [ValueCreg_4 bin2dec(creg_4.bin(9:16))];
          ValueCreg_4 = [ValueCreg_4 bin2dec(creg_4.bin(17:24))];
          
          % change Property in Object
          obj.ValueCreg_4 = ValueCreg_4;
          
%%
%              ConfigParam_05 = struct(...
%              'mult_Hb2',         [ 05 00  24 1]... %
%              )
% Multipliationsfaktor ermöglicht es bei Benutzung von zwei oder mehr
% Halbbrücken die Empfindlichkeit aufeinander anzupassen. Die Werte müssen
% dabei über Messungen mit Nennlast ermittelt werden.
% gewählte Einstellun 2^22 
          creg_5 =   setParam(obj,obj.ConfigParam_05.mult_Hb2,2^22); 
          
          ValueCreg_5 = bin2dec(creg_5.bin(1:8)); 
          ValueCreg_5 = [ValueCreg_5 bin2dec(creg_5.bin(9:16))];
          ValueCreg_5 = [ValueCreg_5 bin2dec(creg_5.bin(17:24))];
          
          % change Property in Object
          obj.ValueCreg_5 = ValueCreg_5;
          
%%
%              ConfigParam_06 = struct(...
%              'mult_Hb3',         [ 06 00  24 1]... %
%              )
% Multipliationsfaktor ermöglicht es bei Benutzung von zwei oder mehr
% Halbbrücken die Empfindlichkeit aufeinander anzupassen. Die Werte müssen
% dabei über Messungen mit Nennlast ermittelt werden.
% gewählte Einstellun 2^22 
          creg_6 =   setParam(obj,obj.ConfigParam_06.mult_Hb3,2^22); 
          
          ValueCreg_6 = bin2dec(creg_6.bin(1:8)); 
          ValueCreg_6 = [ValueCreg_6 bin2dec(creg_6.bin(9:16))];
          ValueCreg_6 = [ValueCreg_6 bin2dec(creg_6.bin(17:24))];
          
          % change Property in Object
          obj.ValueCreg_6 = ValueCreg_6;
          
%%
%              ConfigParam_07 = struct(...
%              'mult_Hb4',         [ 07 00  24 1]... %
%              )
% Multipliationsfaktor ermöglicht es bei Benutzung von zwei oder mehr
% Halbbrücken die Empfindlichkeit aufeinander anzupassen. Die Werte müssen
% dabei über Messungen mit Nennlast ermittelt werden.
% gewählte Einstellun 2^22 
          creg_7 =   setParam(obj,obj.ConfigParam_07.mult_Hb4,2^22); 
          
          ValueCreg_7 = bin2dec(creg_7.bin(1:8)); 
          ValueCreg_7 = [ValueCreg_7 bin2dec(creg_7.bin(9:16))];
          ValueCreg_7 = [ValueCreg_7 bin2dec(creg_7.bin(17:24))];
       
% oder tkpar =1 dann wird Register für virtuellen Parallelwiderstand verwendet          
%           creg_7 =   setParam(obj,obj.ConfigParam_07.mult_TkPar,0);
%           
%           ValueCreg_7 = bin2dec(creg_7.bin(1:8)); 
%           ValueCreg_7 = [ValueCreg_7 bin2dec(creg_7.bin(9:16))];
%           ValueCreg_7 = [ValueCreg_7 bin2dec(creg_7.bin(17:24))];
           
        % change Property in Object
        obj.ValueCreg_7 = ValueCreg_7;

%%
%           ConfigParam_08 = struct(...
%              'TK_Gain',         [ 08 0  24 0x100000],... %
%              )
% ?? noch unklar
          creg_8 =   setParam(obj,obj.ConfigParam_08.Tk_Gain,2^20); 
          
          ValueCreg_8 = bin2dec(creg_8.bin(1:8)); 
          ValueCreg_8 = [ValueCreg_8 bin2dec(creg_8.bin(9:16))];
          ValueCreg_8 = [ValueCreg_8 bin2dec(creg_8.bin(17:24))];
          
          % change Property in Object
          obj.ValueCreg_8 = ValueCreg_8;
          
%%
%           ConfigParam_09 = struct(...
%              'Tk_Offset',        [ 08 0   24 0x100000]... %
%              )
% unsicher Abweichung Maik

          creg_9 =   setParam(obj,obj.ConfigParam_09.Tk_Offset,0); 
          
          ValueCreg_9 = bin2dec(creg_9.bin(1:8)); 
          ValueCreg_9 = [ValueCreg_9 bin2dec(creg_9.bin(9:16))];
          ValueCreg_9 = [ValueCreg_9 bin2dec(creg_9.bin(17:24))];
          
          % change Property in Object
          obj.ValueCreg_9 = ValueCreg_9;
        
%%          
% ConfigParam_10 = struct(...
%              'Mult_Ub',          [ 10 8   8 0xFB],... %
%              'calcor',           [ 10 16  8 0   ],... %
%              'gain_comp',        [ 10 0   8 164 ] ... %
%              )
            
value1 =   setParam(obj,obj.ConfigParam_10.gain_comp,164); 
value2 =   setParam(obj,obj.ConfigParam_10.Mult_Ub,251); 
value3 =   setParam(obj,obj.ConfigParam_10.calcor,20); 

          creg_10 = bitconcat(value3, value2, value1);
                          
          ValueCreg_10 = bin2dec(creg_10.bin(1:8)); 
          ValueCreg_10 = [ValueCreg_10 bin2dec(creg_10.bin(9:16))];
          ValueCreg_10 = [ValueCreg_10 bin2dec(creg_10.bin(17:24))];
          
          % change Property in Object
          obj.ValueCreg_10 = ValueCreg_10;
        
%%
%           ConfigParam_11 = struct(...
%              'io_en_0_sdo',      [ 11 00  2 -1],... %
%              'io_en_1_sdi',      [ 11 02  2 -1],... %
%              'io_en_2_sck',      [ 11 04  2 -1],... %
%              'io_en_3_mio',      [ 11 06  2 -1],... %
%              'io_en_4_mio',      [ 11 08  2 -1],... %
%              'io_en_5_mio',      [ 11 10  2 -1],... %
%              'io_en_6_mio',      [ 11 12  2 -1],... %
%              'io_en_7_mio',      [ 11 14  2 -1],... %
%              'io_en_digital',    [ 11 16  8 1]  ... %
%              )
% ! Abweichung Maik: io_en_digital

         
        value1 =   setParam(obj,obj.ConfigParam_11.io_en_0_sdo,3); 
        value2 =   setParam(obj,obj.ConfigParam_11.io_en_1_sdi,3); 
        value3 =   setParam(obj,obj.ConfigParam_11.io_en_2_sck,3); 
        value4 =   setParam(obj,obj.ConfigParam_11.io_en_3_mio,0); 
        value5 =   setParam(obj,obj.ConfigParam_11.io_en_4_mio,0); 
        value6 =   setParam(obj,obj.ConfigParam_11.io_en_5_mio,0); 
        value7 =   setParam(obj,obj.ConfigParam_11.io_en_6_mio,0); 
        value8 =   setParam(obj,obj.ConfigParam_11.io_en_7_mio,0); 
        value9 =   setParam(obj,obj.ConfigParam_11.io_en_digital,1); 

        creg_11 = bitconcat( value9,  value8,  ...
                             value7,  value6,  value5,  ...
                             value4,  value3,  value2,  ...
                             value1);
                          
        ValueCreg_11 = bin2dec(creg_11.bin(1:8)); 
        ValueCreg_11 = [ValueCreg_11 bin2dec(creg_11.bin(9:16))];
        ValueCreg_11 = [ValueCreg_11 bin2dec(creg_11.bin(17:24))];
        
        % change Property in Object
        obj.ValueCreg_11 = ValueCreg_11;
          
%%
%                   ConfigParam_12 = struct(...
%              'con_comp',              [ 12 18  2 1 ],... %
%              'dis_pp_cycle_mod',      [ 12 21  1 1 ],... %
%              'ext_eeprom_clk_speed',  [ 12 16  2 1 ],... %
%              'en_emi_noise_reduction',[ 12 07  1 1 ],... %
%              'irq_dsp_edge',          [ 12 15  1 0 ],... %
%              'irq_dsp_en',            [ 12 14  1 -1],... %
%              'irq_dsp_pin_sel',       [ 12 23  1 0 ],... %
%              'lcd_clk_open_drain',    [ 12 6   1 0 ],... %
%              'lcd_clk_pol',           [ 12 22  1 0 ],... %
%              'lcd_clk_sel',           [ 12 10  2 3 ],... %
%              'sel_comp_int',          [ 12 20  1 -1],... %
%              'multio3_sel',           [ 12 12  2 -1],... %
%              'sel_refresh_vlt',       [ 12 8   2  2],... %
%              'usr_epr_always_on',     [ 12 5   1  0],... %
%              'usr_epr_prg_time',      [ 12 2   2  0] ... % 
%              )
        value1 =   setParam(obj,obj.ConfigParam_12.acam_internal_bit1,0); 
        value2 =   setParam(obj,obj.ConfigParam_12.usr_epr_prg_time,3); 
        value3 =   setParam(obj,obj.ConfigParam_12.acam_internal_bit2,3); 
        value4 =   setParam(obj,obj.ConfigParam_12.usr_epr_always_on,3); 
        value5 =   setParam(obj,obj.ConfigParam_12.lcd_clk_open_drain,0); 
        value6 =   setParam(obj,obj.ConfigParam_12.en_emi_noise_reduction,0); 
        value7 =   setParam(obj,obj.ConfigParam_12.sel_refresh_vlt,2); 
        value8 =   setParam(obj,obj.ConfigParam_12.lcd_clk_sel,0); 
        value9 =   setParam(obj,obj.ConfigParam_12.multio3_sel,1); 
        value10 =   setParam(obj,obj.ConfigParam_12.irq_dsp_en,0); 
        value11 =   setParam(obj,obj.ConfigParam_12.irq_dsp_edge,0);
        value12 =   setParam(obj,obj.ConfigParam_12.ext_eeprom_clk_speed,1);
        value13 =   setParam(obj,obj.ConfigParam_12.con_comp,1);
        value14 =   setParam(obj,obj.ConfigParam_12.sel_comp_int,1);
        value15 =   setParam(obj,obj.ConfigParam_12.dis_pp_cycle_mod,1);
        value16 =   setParam(obj,obj.ConfigParam_12.lcd_clk_pol,0);
        value17 =   setParam(obj,obj.ConfigParam_12.irq_dsp_pin_sel,0);


        creg_12 = bitconcat( value17, value16, value15, value14, value13, ...
                             value12, value11, value10, ...
                             value9,  value8, value7, ...
                             value6,  value5, value4, ...
                             value3,  value2, value1);
                          
        ValueCreg_12 = bin2dec(creg_12.bin(1:8)); 
        ValueCreg_12 = [ValueCreg_12 bin2dec(creg_12.bin(9:16))];
        ValueCreg_12 = [ValueCreg_12 bin2dec(creg_12.bin(17:24))];
        
        % change Property in Object
        obj.ValueCreg_12 = ValueCreg_12;
        
% ConfigParam_13 = struct(...
%              'crf_sen2',         [ 13 1   2 10 ],... %
%              'crf_sen3',         [ 13 22  1  0 ],... %
%              'crf_tau',          [ 13 18  1  0 ],... %
%              'crf_tp1',          [ 13 20  1 0  ],... %
%              'mi_enable',        [ 13 11  3 -1 ],... %
%              'multio4_sel',      [ 13 00  3 -1 ],... %
%              'mi_sel_clk5k',     [ 13 16  1 0  ],... %
%              'mi_updaterate',    [ 13 14  2 1  ],... %
%              'sel_rc_osc2',      [ 13 17  1 -1 ],... %
%              'sel_rc_osc1',      [ 13 19  1 -1 ],... %
%              'store_tdc_times',  [ 13 23  1 0  ],... %
%              'selqha',           [ 13 4   6  45],... %
%              'sense_discharge',  [ 13 10  1 1  ] ... %
%              )
%         

value1 =   setParam(obj,obj.ConfigParam_13.multio4_sel,0);
value2 =   setParam(obj,obj.ConfigParam_13.selqha,45);
value3 =   setParam(obj,obj.ConfigParam_13.sense_discharge,1);
value4 =   setParam(obj,obj.ConfigParam_13.mi_enable,0);
value5 =   setParam(obj,obj.ConfigParam_13.mi_updaterate,0);
value6 =   setParam(obj,obj.ConfigParam_13.mi_sel_clk5k,0);
value7 =   setParam(obj,obj.ConfigParam_13.sel_rc_osc2,0);
value8 =   setParam(obj,obj.ConfigParam_13.crf_tau,0);
value9 =   setParam(obj,obj.ConfigParam_13.sel_rc_osc1,1);
value10 =   setParam(obj,obj.ConfigParam_13.crf_tp1,0);
value11 =   setParam(obj,obj.ConfigParam_13.crf_sen2,0);
value12 =   setParam(obj,obj.ConfigParam_13.crf_sen3,0);
value13 =   setParam(obj,obj.ConfigParam_13.store_tdc_times,0);


        creg_13 = bitconcat( value13, ...
                             value12, value11, value10, ...
                             value9,  value8, value7, ...
                             value6,  value5, value4, ...
                             value3,  value2, value1);
                          
        ValueCreg_13 = bin2dec(creg_13.bin(1:8)); 
        ValueCreg_13 = [ValueCreg_13 bin2dec(creg_13.bin(9:16))];
        ValueCreg_13 = [ValueCreg_13 bin2dec(creg_13.bin(17:24))];
        
        % change Property in Object
        obj.ValueCreg_13 = ValueCreg_13;
        
        
%         ConfigParam_14 = struct(...
%                 'sel_rtemp_300R',   [ 14 0   1 -1 ],... %
%                 'caltime',          [ 14 1   2 1  ],... %
%                 'alupernopen',      [ 14 3   1 3  ],... %
%                 'internalBits',     [ 14 5   4 1  ],... %
%                 'acam_internal',    [ 14 8   4 1  ],... %
%                 'upd_vlt',          [ 14 12  3 0  ], ... %
%                 'en_pp_measurement',[ 14 15  1 1  ],... %
%                 'iternal_1',        [ 14 15  3 0  ],... %
%                 'bandgap_trim',     [ 14 19  4 10 ],... % 
%                 'iternal_2',        [ 14 23  1 0  ]... %
%              )
% Abweichung von Maik da caltime =1

        value1 =   setParam(obj,obj.ConfigParam_14.sel_rtemp_300R,1);
        value2 =   setParam(obj,obj.ConfigParam_14.caltime,1);
        value3 =   setParam(obj,obj.ConfigParam_14.alupernopen,0);
        value4 =   setParam(obj,obj.ConfigParam_14.internalBits,3);
        value5 =   setParam(obj,obj.ConfigParam_14.acam_internal,1);
        value6 =   setParam(obj,obj.ConfigParam_14.upd_vlt,1);
        value7 =   setParam(obj,obj.ConfigParam_14.en_pp_measurement,1);
        value8 =   setParam(obj,obj.ConfigParam_14.iternal_1,0);
        value9 =   setParam(obj,obj.ConfigParam_14.bandgap_trim,10);
        value10 =  setParam(obj,obj.ConfigParam_14.iternal_2,0);

        creg_14 = bitconcat( value10, ...
                             value9,  value8, value7, ...
                             value6,  value5, value4, ...
                             value3,  value2, value1);
                          
        ValueCreg_14 = bin2dec(creg_14.bin(1:8)); 
        ValueCreg_14 = [ValueCreg_14 bin2dec(creg_14.bin(9:16))];
        ValueCreg_14 = [ValueCreg_14 bin2dec(creg_14.bin(17:24))];
        
        % change Property in Object
        obj.ValueCreg_14 = ValueCreg_14;
        
%           ConfigParam_15 = struct(...
%              'cport_adapt_speed',  [ 15 0   8 1 ],... %
%              'cport_thresh2',      [ 15 8   8 -1 ],... %
%              'cport_en',           [ 15 16  4 0  ],... %
%              'cport_r',            [ 15 20  2 0  ],... %
%              'cport_update',       [ 15 22  1 0  ], ... %
%              'cport_adapt',        [ 15 23  1 1  ]... %
%              )

        value1 =   setParam(obj,obj.ConfigParam_15.cport_adapt_speed,1);
        value2 =   setParam(obj,obj.ConfigParam_15.cport_thresh2,100);
        value3 =   setParam(obj,obj.ConfigParam_15.cport_en,0);
        value4 =   setParam(obj,obj.ConfigParam_15.cport_r,2);
        value5 =   setParam(obj,obj.ConfigParam_15.cport_update,0);
        value6 =   setParam(obj,obj.ConfigParam_15.cport_adapt,1);
        
        creg_15 = bitconcat( value6,  value5, value4, ...
                             value3,  value2, value1);
                          
        ValueCreg_15 = bin2dec(creg_15.bin(1:8)); 
        ValueCreg_15 = [ValueCreg_15 bin2dec(creg_15.bin(9:16))];
        ValueCreg_15 = [ValueCreg_15 bin2dec(creg_15.bin(17:24))];
        
        % change Property in Object
        obj.ValueCreg_15 = ValueCreg_15;
        
        end
       
        function obj = FastInit(obj,Version)
            
            if strcmp(Version,'V1')
                obj.ValueCreg_0 = [10,0,210];
                obj.ValueCreg_1 = [172,69,1];
                obj.ValueCreg_2 = [85,68,122];
                obj.ValueCreg_3 = [130,1,120];
                obj.ValueCreg_4 = [64,0,0];
                obj.ValueCreg_5 = [64,0,0];
                obj.ValueCreg_6 = [64,0,0];
                obj.ValueCreg_7 = [64,0,0];
                obj.ValueCreg_8 = [16,0,0];
                obj.ValueCreg_9 = [0,0,0];
                obj.ValueCreg_10 = [20,251,164];
                obj.ValueCreg_11 = [1,0,63];
                obj.ValueCreg_12 = [53,18,60];
                obj.ValueCreg_13 = [8,6,208];
                obj.ValueCreg_14 = [80,145,51];
                obj.ValueCreg_15 = [160,100,1];
            elseif strcmp(Version,'V2')
                % Werte aus DA Maik Häse
                obj.ValueCreg_0 = [0x23,0x00,0xD2];
                obj.ValueCreg_1 = [0x2C,0x45,0xC0];
                obj.ValueCreg_2 = [0x55,0x44,0x72];
                obj.ValueCreg_3 = [0x82,0x02,0x78];
                obj.ValueCreg_4 = [0x40,0x00,0x00];
                obj.ValueCreg_5 = [0x40,0x00,0x00];
                obj.ValueCreg_6 = [0x40,0x00,0x00];
                obj.ValueCreg_7 = [0x40,0x00,0x00];
                obj.ValueCreg_8 = [0x10,0x00,0x00];
                obj.ValueCreg_9 = [0x10,0x00,0x00];
                obj.ValueCreg_10 = [0x14,0xFB,0xA4];
                obj.ValueCreg_11 = [0x00,0x00,0x3F];
                obj.ValueCreg_12 = [0x35,0x12,0x00];
                obj.ValueCreg_13 = [0x0C,0x05,0x40];
                obj.ValueCreg_14 = [0x60,0xA1,0x33];
                obj.ValueCreg_15 = [0x80,0x64,0x01];
            else
                
                
            end
        
        end
        
        function obj= write_config(obj)
            instrhwinfo('i2c','NI845x')
             
            obj.i2cobj = i2c('NI845x', 0, '60h');   % Erstelle I²C-Objekt Vendor 'NI845x', BoardIndex 0, Remote-Adresse 60h
            obj.i2cobj.BitRate = 100;
            obj.i2cobj.ByteOrder = 'littleEndian';
            obj.i2cobj.PullupResistors = 'both';
                                                    
            fopen(obj.i2cobj);                      % Öffne Verbindung USB-845x adapter.
            
            fwrite(obj.i2cobj, [obj.i2cWrite, obj.power_reset]);
            fwrite(obj.i2cobj, obj.watchdog_off);
            
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg0 obj.ValueCreg_0 ] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg1 obj.ValueCreg_1 ] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg2 obj.ValueCreg_2 ] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg3 obj.ValueCreg_3 ] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg4 obj.ValueCreg_4 ] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg5 obj.ValueCreg_5 ] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg6 obj.ValueCreg_6 ] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg7 obj.ValueCreg_7 ] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg8 obj.ValueCreg_8 ] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg9 obj.ValueCreg_9 ] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg10 obj.ValueCreg_10] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg11 obj.ValueCreg_11] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg12 obj.ValueCreg_12] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg13 obj.ValueCreg_13] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg14 obj.ValueCreg_14] );
            fwrite(obj.i2cobj, [ obj.i2cWrite obj.c_reg15 obj.ValueCreg_15] );
            
            fwrite(obj.i2cobj, obj.init_reset);
            fwrite(obj.i2cobj, obj.start_newcycle);
            
            %% check Config Register on PS09
            
            %ACHTUNG!! Bei aktuellem Aufbau kommt es oft zu einem Wackelkontakt
            
            error = '';
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg0]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_0')
            else
                error = [error 'ValueCreg0'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg1]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_1')
            else
                error = [error 'ValueCreg1'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg2]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_2')
            else
                error =  [error 'ValueCreg2'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg3]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_3')
            else
                error = [error 'ValueCreg3'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg4]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_4')
            else
                error = [error 'ValueCreg4'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg5]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_5')
            else
                error = [error 'ValueCreg5'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg6]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_6')
            else
                error = [error 'ValueCreg6'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg7]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_7')
            else
                error = [error 'ValueCreg7'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg8]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_8')
            else
                error = [error 'ValueCreg8'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg9]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_9')
            else
                error = [error 'ValueCreg9'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg10]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_10')
            else
                error = [error 'ValueCreg10'];
            end
        
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg11]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_11')
            else
                error = [error 'ValueCreg11'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg12]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_12')
            else
                error = [error 'ValueCreg12'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg13]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_13')
            else
                error = [error 'ValueCreg13'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg14]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_14')
            else
                error = [error 'ValueCreg14'];
            end
            
            fwrite(obj.i2cobj,[obj.i2cRead obj.c_reg15]);
            if  all(fread (obj.i2cobj, 3) == obj.ValueCreg_15')
            else
                error = [error 'ValueCreg15'];
            end
            
            disp(error);
            
            
            
        end
        

        
        function value = setParam(obj,Param,value)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
                
            if value ~= Param(4)
                value = value; %Param(4);
            end
            
          
             value = fi(value,false,Param(3),0);
         
            
        end
    end
end

