Global Integer i
Global Integer H1(14)
Global Integer H2(14)

Function Grip_On       ' activar pinza (lógica NEGATIVA: activo = nivel bajo)
    Off Out_9
Fend

Function Grip_Off      ' desactivar pinza (lógica NEGATIVA: inactivo = nivel alto)
    On Out_9
Fend

Function main
	Motor On
	Power Low
	Accel 100, 100 '%
	Speed 100 '%	
	Grip_Off()
	Home
	Call Paletizado_01
	Call Movimiento_huevos()
	Home
Fend
Function Paletizado_01
	Pallet 1, Origen, PuntoX, PuntoY, 6, 5
	H1(0) = 1
	H1(1) = 9
	H1(2) = 5
	H1(3) = 18
	H1(4) = 29
	H1(5) = 21
	H1(6) = 25
	H1(7) = 14
	H1(8) = 3
	H1(9) = 7
	H1(10) = 20
	H1(11) = 16
	H1(12) = 12
	H1(13) = 23
	H1(14) = 27
	
	H2(0) = 30
	H2(1) = 17
	H2(2) = 6
	H2(3) = 10
	H2(4) = 2
	H2(5) = 13
	H2(6) = 26
	H2(7) = 22
	H2(8) = 11
	H2(9) = 24
	H2(10) = 28
	H2(11) = 15
	H2(12) = 4
	H2(13) = 8
	H2(14) = 19
	
Fend

Function Movimiento_huevos
	For i = 1 To 14
			Jump Pallet(1, H1(i - 1))
			Grip_On()
			Wait 1
			Jump Pallet(1, H1(i))
			Grip_Off()
			Jump Pallet(1, H2(i - 1))
		    Grip_On()
		    Wait 1
			Jump Pallet(1, H2(i))
			Grip_Off()
		Next
		Jump Pallet(1, H1(14))
		Grip_On()
		Wait 1
		Jump Pallet(1, H2(0))
		Grip_Off()
		Jump Pallet(1, H2(14))
		Grip_On()
		Wait 1
		Jump Pallet(1, H1(0))
		Grip_Off()
    For i = 1 To 14
			Jump Pallet(1, H1(i - 1))
			Grip_On()
			Wait 1
			Jump Pallet(1, H1(i))
			Grip_Off()
			Jump Pallet(1, H2(i - 1))
		    Grip_On()
		    Wait 1
			Jump Pallet(1, H2(i))
			Grip_Off()
		Next
Fend
