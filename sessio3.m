%////////////////////
% sessio3.m
%////////////////////

function sessio3(serPort)

		%bug1(serPort,[3.5,-1]);
			%bug1(serPort,[-4,1]);
		%bug1(serPort,[1, -4]);
		bug1(serPort,[-5,-4]);
		
	
		function bug1(serPort,objectiu)
			obstacle=false;
			%[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
			%linia = (objectiu - [x, y]);
			[BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...     
                 BumpsWheelDropsSensorsRoomba(serPort);
            [x, y, anguloRads]=OverheadLocalizationCreate(serPort);
			DecisionAnguloGiro(x, y, anguloRads,objectiu);



			while ~hayObstaculo() && ~hemArribat([x, y], objectiu)
				fprintf('bucle principal');
				[x, y]=OverheadLocalizationCreate(serPort);
				 %[BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...     
                 %BumpsWheelDropsSensorsRoomba(serPort);
                 %[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
				 SetDriveWheelsCreate(serPort,.5,.5);
				 pause(0.000001);
			end 

			if hayObstaculo()
				SetDriveWheelsCreate(serPort,.0,.0);
				
			end
			followBoundary(serPort,objectiu);


		end
		%% ERROR!!!
		function preFollowBoundary ()
			distancia=[];
			angulos=[];
			[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           	anguloActual=pasarAGrados(anguloRads);


			fprintf('Inicializamos preFollowBoundary');
			turnAngle(serPort, .2,90);
			distancia(1)= ReadSonarMultiple(serPort,1);
			[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           	angulos(1)=adaptarGrados(pasarAGrados(anguloRads));

			turnAngle(serPort, .2,90);
			distancia(2)= ReadSonarMultiple(serPort,1)
			[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           	angulos(2)=adaptarGrados(pasarAGrados(anguloRads));

			turnAngle(serPort, .2,90);
			distancia(3)= ReadSonarMultiple(serPort,1)
			[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           	angulos(3)=adaptarGrados(pasarAGrados(anguloRads));

			turnAngle(serPort, .2,90);
			distancia(4)= ReadSonarMultiple(serPort,1)
			[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           	angulos(4)=adaptarGrados(pasarAGrados(anguloRads))
           	
           	indice=find(distancia==min(distancia));
           	anguloDestino=angulos(indice)
           	[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           	anguloSensor=pasarAGrados(anguloRads);
           	anguloSensor=adaptarGrados(anguloSensor)
           	anguloDestino=valorAbsoluto((360-anguloSensor)+anguloDestino);
           	fprintf('el angulo hal que hay que ir es ');
           	anguloDestino
           	turnAngle(serPort, .2,anguloDestino);
           	beep();
			

           	%recalcularAngulo(serPort);
	
			
		end
				
		
		
		function followBoundary(serPort,objectiu)
			fprintf('Inicializamos FollowBoundary');
			preFollowBoundary();


            
		end

		function distancia=hemArribat(posicioActual,objectiu)

			 distancia=sqrt( ((objectiu(1)-posicioActual(1)).^2)...
						    + ((objectiu(2)-posicioActual(2)).^2)...
						   )
			 % distancia;
			 if distancia < 0.3
			 	distancia=true;
			 else 
			 	distancia=false;
			 end
		end
		
		
		function valor=Entre(sensor,Valorinferior,Valorsuperior)
			if sensor > Valorinferior && sensor < Valorsuperior
				valor=1;% se cumple
			else
				valor=0;
			end

		end

		function calcularFrenada(serPort)
			for i=0:200
				%fprintf('frenoooooo\n');
           		 		pause(0.0001);
           		 		SetDriveWheelsCreate(serPort, .2,.2);
           		 		
           	end

		end
			function trobat=hayObstaculo()
				 trobat=false;
				 distDerecha= ReadSonarMultiple(serPort,1);
           		 distFrontal = ReadSonarMultiple(serPort,2);
           		 distIzquierda = ReadSonarMultiple(serPort,3);
				if distDerecha < 0.2 || distIzquierda < 0.2  || distFrontal < 0.2 
					trobat=true; 
				else
					trobat =false;
				end

			end
		function StopCreate(serPort) 
		        % Stop the robot 
		        % serPort is the serial port number (for controlling the actual robot). 
		        SetDriveWheelsCreate(serPort, 0,0)  
		end
		function valor=valorAbsoluto(valor)
			if valor < 0
				valor=-valor
			else
				valor=valor
			end
		end

		
		function grados=pasarAGrados(angulo)
			angulo=double(angulo);
			grados=double(angulo*(180/pi));
		end
		function DecisionAnguloGiro(x, y, anguloRads,objectiu)
			% x , y->>>> posicion actual
			%esta funcion decide hace donde girar el robot para que haga 
			%el minimo giro posible y ademas se mueva hacia el angulo que 
			%forma la linia mas corta hacia el objetivo atan(angulo)
			% donde anguloRads es el angulo actual del robot en sentio antihorario
			% donde objectiu es el punto x e y del punto final al que llegar
			%donde angulo giro sera el angulo al que girara el robot hacia el objetivo
			anguloGiro=0
			catetoContiguo=0%para calcular el angulo
			catetoOpuesto=0%para calcular el angulo

			anguloActual=pasarAGrados(anguloRads);
			if objectiu(1) >= 0 && objectiu(2) >= 0%primer cuadrante	
				catetoContiguo=objectiu(1)-x;
				catetoOpuesto=objectiu(2)-y;
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro);
				anguloGiro=abs(anguloGiro)
				turnAngle(serPort, .2,anguloGiro);
			elseif objectiu(1) < 0 && objectiu(2) >= 0%segundo cuadrante
				catetoContiguo=abs(objectiu(1)-x);
				catetoOpuesto=abs(objectiu(2)-y);
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro);
				anguloGiro=abs(180-anguloGiro)
				turnAngle(serPort, .2,anguloGiro);
			elseif objectiu(1)<0 && objectiu(2) < 0 %tercer cuadrande 
				catetoContiguo=abs(objectiu(1)-x);
				catetoOpuesto=abs(objectiu(2)-y);
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro)
				anguloGiro=abs(180+anguloGiro)
				turnAngle(serPort, .2,anguloGiro);
			else                                    %cuarto cuadrande
				catetoContiguo=abs(objectiu(1)-x);
				catetoOpuesto=abs(objectiu(2)-y);
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro)
				anguloGiro=abs(anguloGiro)
				
				turnAngle(serPort, .2,-anguloGiro);
			end


		end
		function convertido=adaptarGrados(valor)
			%esta funcion coge el valor i si es negativo lo 
			%adapta para que el valor este en un intervalo de 
			%0 a 360 no de 0 a 180 i -180 a 0
			if valor < 0
				convertido=-valor-180;
				convertido=-convertido+180;
			else
				convertido=valor;
			end
		end


			%%%%%%%%%%%%%%%%%%%%%%%%
		function Signal()
			% Make signal sound (4 beeps)
			n= 4;
			for k=1:4
				beep
				pause(1)
			end
		end
end

