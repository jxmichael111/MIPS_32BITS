-------------------------------------------------------------------------------
--
-- Title       : MIPS
-- Design      : proyect
-- Author      : Michael Jarnie Ticona Larico
-- Company     : CS
--
-------------------------------------------------------------------------------
--
-- File        : MIPS.vhd
-- Generated   : Fri Aug  4 02:07:23 2023
-- From        : interface description file
-- By          : Itf2Vhdl ver. 1.20
--
-------------------------------------------------------------------------------
--
-- Description : 
--
-------------------------------------------------------------------------------

--{{ Section below this comment is automatically maintained
--   and may be overwritten
--{entity {MIPS} architecture {MIPS}}

library IEEE;
use IEEE.STD_LOGIC_1164.all; 
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity MIPS is 
	port(
    Instruccion  : in  STD_LOGIC_VECTOR (31 downto 0)
  );
end MIPS;

--}} End of automatically maintained section

architecture MIPS of MIPS is	
	-- señales de la memoria de instruccion
    signal Op   : STD_LOGIC_VECTOR (5 downto 0);	-- instucciones (31 - 26)
    signal Instr25_21   : STD_LOGIC_VECTOR (4 downto 0);
    signal Instr20_16   : STD_LOGIC_VECTOR (4 downto 0);
    signal Instr15_11   : STD_LOGIC_VECTOR (4 downto 0);
    signal Instr15_0    : STD_LOGIC_VECTOR (15 downto 0);	
	signal Instr5_0    : STD_LOGIC_VECTOR (5 downto 0);
	-- señales de la unidad de control
	signal RegDest :  std_logic; 
    signal FuenteALU :  std_logic; 
    signal MemtoReg :  std_logic; 
    signal EscribirReg :  std_logic; 
    signal LeerMem,EscribirMem :  std_logic; 
    signal Salto :  std_logic; 
    signal ALUOp:  std_logic_vector(1 downto 0); 
	signal TipoR, Lw, Sw, beq : std_logic;	 
	
	-- Registros  
	signal Registro_lectura_1 	:  STD_LOGIC_VECTOR (4 downto 0);
    signal Registro_lectura_2 	:  STD_LOGIC_VECTOR (4 downto 0);
    signal Reg_escritura 		:  STD_LOGIC_VECTOR (4 downto 0);
    signal Escribir_dato 		:  STD_LOGIC_VECTOR (31 downto 0);
    signal Dato_leido_1 		:  STD_LOGIC_VECTOR (31 downto 0);
    signal Dato_leido_2 		:  STD_LOGIC_VECTOR (31 downto 0);
type rom is array (0 to 31) of STD_LOGIC_VECTOR(31 downto 0);
signal registros: rom := (
    x"00000000", -- 0  $zero
    x"00000000", -- 1  $at
    x"00000100", -- 2  $v0
    x"00000A00", -- 3  $v1
    x"00000A00", -- 4  $a0
    x"00000000", -- 5  $a1
    x"00000A00", -- 6  $a2
    x"00000000", -- 7  $a3
    x"00000000", -- 8  $t0
    x"00000000", -- 9  $t1
    x"00000000", -- 10 $t2
    x"00000000", -- 11 $t3
    x"00000000", -- 12 $t4
    x"00000000", -- 13 $t5
    x"00000000", -- 14 $t6
    x"00000000", -- 15 $t7
    x"00000000", -- 16 $s0
    x"00000000", -- 17 $s1
    x"00000000", -- 18 $s2
    x"00000000", -- 19 $s3
    x"00000000", -- 20 $s4
    x"00000000", -- 21 $s5
    x"00000000", -- 22 $s6
    x"00000000", -- 23 $s7
    x"00000000", -- 24 $t8
    x"00000000", -- 25 $t9
    x"00000000", -- 26 $k0
    x"00000000", -- 27 $k1
    x"00000000", -- 28 $gp
    x"00000000", -- 29 $sp
    x"00000000", -- 30 $fp
    x"00000000"  -- 31 $ra
);

-- extensor de signo  
	signal Instr15_0_extended : std_logic_vector(31 downto 0);
    signal extension : std_logic_vector(15 downto 0) := (others => '0');   

--	control del alu
	signal FUNCT		:	std_logic_vector(5 downto 0);	-- Campo de la instrucción FUNC
	signal ALU_CONTROL	:	std_logic_vector(2 DOWNTO 0);	
-- ALU
	signal X			: std_logic_vector(31 downto 0);
	signal Y			: std_logic_vector(31 downto 0);
	signal R	: std_logic_vector(31 downto 0);  --RESULTADO DEL ALU
	signal CIN   	: std_logic;	  --CARRY_IN
	signal COUT   	: std_logic;	--CARRY_OUT
	
	component FULL_ADDER is
	    port(
			X	: in	STD_LOGIC;
			Y	: in	STD_LOGIC;
			CIN	: in	STD_LOGIC;
			COUT	: out	STD_LOGIC;
			R	: out	STD_LOGIC
	    );
	end component FULL_ADDER;
 	signal CAUX : STD_LOGIC_VECTOR (31 downto 0); --AUXILIAR SUMA 
	signal SAUX : STD_LOGIC_VECTOR (31 downto 0);  -- RESULTADO DE LA SUMA Y RESTA	 
	-- 	MEMORIA DE DATOS
	signal RESET         : STD_LOGIC;                --Reset 
    signal DIRECCION     : STD_LOGIC_VECTOR (31 downto 0);    --Dirección a ser leida o escrita
    signal WRITE_DATA  	 : STD_LOGIC_VECTOR (31 downto 0);    --Datos a ser escritos
    signal DATO_LEIDO    : STD_LOGIC_VECTOR (31 downto 0);    --Datos leidos  
	
	type MEM_T is array (1023 downto 0) of STD_LOGIC_VECTOR (31 downto 0);
	signal MEM : MEM_T;
	
	
begin

	-- asignacion de las intrucciones divididas -- 
	Op <= Instruccion(31 downto 26);
  	Instr25_21 <= Instruccion(25 downto 21);
 	Instr20_16 <= Instruccion(20 downto 16);
 	Instr15_11 <= Instruccion(15 downto 11);
  	Instr15_0  <= Instruccion(15 downto 0);
	Instr5_0   <= Instruccion(5 downto 0);
	  
	-- aginacion de las señales de cada tipo de instruccion --
	TipoR <= not Op(5) and not Op(4) and not Op(3) and not Op(2) and not Op(1) and not Op(0);
    Lw <= Op(5) and not Op(4) and not Op(3) and not Op(2) and Op(1) and Op(0);
    Sw <= Op(5) and not Op(4) and Op(3) and not Op(2) and Op(1) and Op(0);
    beq <= not Op(5) and not Op(4) and not Op(3) and Op(2) and not Op(1) and not Op(0);
	
	-- control de los diferentes tipos de señales
    RegDest <= TipoR;
    FuenteALU <= Lw or Sw;
    MemtoReg <= Lw;
    EscribirReg <= TipoR or Lw;
    LeerMem <= Lw;
    EscribirMem <= Sw;
    Salto <= beq;
    ALUOp(1) <= TipoR;
    ALUOp(0) <= beq; 
	
	-- obtencion de los registros leidos   
	Registro_lectura_1 <= Instr25_21;
	Registro_lectura_2 <= Instr20_16;
	Dato_leido_1 <= registros(conv_integer(Registro_lectura_1)) ;
	Dato_leido_2 <= registros(conv_integer(Registro_lectura_2)) ;
	
	--mux de regDest
	with RegDest select	
	Reg_escritura <= Instr20_16 when '0',
	Instr15_11 when others,; 
	
	--extensor de signo
	Instr15_0_extended <= extension & Instr15_0; 	  --los concatenamos  
	
	--control de la ALU	   

	FUNCT <=  Instr5_0;
	ALU_CONTROL(2) <= ALUOp(0) or (ALUOp(1) and FUNCT(2)); 
	ALU_CONTROL(1) <= not(ALUOp(1) and FUNCT(2)); 
	ALU_CONTROL(0) <= ALUOp(1) and (FUNCT(0) OR FUNCT(3)); 	
	
	--mux de FuenteALU
	X <= Dato_leido_1 ;
with FuenteALU select	
	Y <= Dato_leido_2 when '0',
		Instr15_0_extended  when others,;   
	
	--OPERACIONES DEL ALU 
    CIN <= '0';  
	BEGIN_FA:
		FULL_ADDER port map (
			X	=> X(0),
			Y	=> Y(0),
			CIN	=> CIN,
			COUT	=> CAUX(0),
			R	=> SAUX(0)
		);
	GEN_ADDER:
		for i in 1 to 31 generate
			NEXT_FA:
				FULL_ADDER port map (
					X	=> X(i),
					Y	=> Y(i),	
					CIN	=> CAUX(i-1),
					COUT=> CAUX(i),
					R	=> SAUX(i)
				); 
			COUT <= CAUX(31);
		end generate;
		
		--OPERACIONES REALIZADAS POR EL ALU
with ALU_CONTROL select	
		R <= SAUX when "010",
			 SAUX when "110",
	 		 X  and Y	when "000",
			 X  or Y when "001",
			 x"00000000" when others,;  
	--MEMORIA DE DATOS	   
	RESET <= '1';
MEM_PROC:
		process(RESET,EscribirMem,LeerMem,ESCRIBIR_DATO,MEM,DIRECCION)
		begin	
			if (RESET = '1') then -- Reset Asincrónico
				for i in 0 to 1023 loop	
					MEM(i) <= (others => '1');
				end loop;
			 -- Ejecuto las ordenes de la unidad de control:
			elsif EscribirMem='1' then -- O bien escribo en la memoria
				MEM(conv_integer(unsigned( DIRECCION (9 downto 0) ))) <= WRITE_DATA;
			elsif LeerMem='1' then -- O bien leo de ella
			    	DATO_LEIDO <= MEM(conv_integer(unsigned( DIRECCION (9 downto 0) )));
			end if;
		end process MEM_PROC;	
	
	--MUX DE MENTOREG
with MemtoReg select	
	Escribir_dato <= R when '0',
					 DATO_LEIDO   when others,; 
				
proceso_registro: process(EscribirReg)
    begin
        if EscribirReg = '1' then
            registros(conv_integer (unsigned(Reg_escritura))) <= Escribir_dato;
        end if;
    end process proceso_registro;


end MIPS;
