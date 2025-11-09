ver1(10/14): Area:19000, cycle time:3.70ns, PI = 70300 // no pipeline

===== FF after rotorB inv ====
ver2(10/15): Area:22313, cycle time:2.77ns, PI = 61807 // 2 stage pipeline 
ver3(10/15): Area:20389, cycle time:2.77ns, PI = 56477 // 2 stage pipeline + simplify plugboard
ver4(10/15): Area:18550, cycle time:2.27ns, PI = 42108 // 2 stage pipeline + simplify plugboard & rotorA
ver5(10/15): Area:18405, cycle time:2.267ns, PI = 41724 // 2 stage pipeline + simplify plugboard & rotorA & some small parts
ver6(10/15): Area:18317, cycle time:2.18ns, PI = 39931 // 2 stage pipeline + simplify plugboard & rotorA & some parts
ver7(10/20): Area:16329, cycle time:2.24ns, PI = 36577 // 2 stage pipeline + simplify plugboard & rotorA & some parts & move the rotorA_sel_mux
ver8(10/20): Area:16305, cycle time:2.221ns, PI = 36213 // 2 stage pipeline + simplify plugboard & rotorA & some parts & move the rotorA_sel_mux & merge bit_sw_mode_sel

===== Hidden pattern failed ====
ver9(10/23): Area:16212, cycle time:2.1819ns, PI = 35373 // 2 stage pipeline + simplify plugboard & rotorA & some parts & move the rotorA_sel_mux & merge bit_sw_mode_sel & small part

ver10(10/23): Area:16173, cycle time:2.1705ns, PI = 35103 // 2 stage pipeline + simplify plugboard & rotorA & some parts & move the rotorA_sel_mux & merge bit_sw_mode_sel & small part & rotorA part
