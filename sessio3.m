%////////////////////
% sessio3.m
%////////////////////

function sessio3(serPort)

		bug1(serPort,[2,5]);
		function bug1(serPort,objectiu)
			obstacle=false;
			[BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...     
                 BumpsWheelDropsSensorsRoomba(serPort);
            [x, y, anguloRads]=OverheadLocalizationCreate(serPort);
			DecisionAnguloGiro(x, y, anguloRads,objectiu);
			while ~hayObstaculo() && ~hemArribat([x, y], objectiu)
				fprintf('bucle principal');
				[x, y]=OverheadLocalizationCreate(serPort);
				 SetDriveWheelsCreate(serPort,.5,.5);
				 pause(0.000001);
			end 
			if hayObstaculo()
				SetDriveWheelsCreate(serPort,.0,.0);
				
			end
			followBoundary(serPort,objectiu);
		end
		function preFollowBoundary()
			%la idea de esta funcion es una vez nos encontramos un obstaculo lo que 
			%lo que queremos es que se quede a la derecha del robot para poder empezar
			%a rodearlo por la derecha por lo tanto la idea para realizar esto es 
			%dar una vuelta de unos 360 grados para poder ir guardando en un vector los 
			%puntos en los que el sensor derecha devuelva una distancia mas cercana

			distancia=[];
			angulos=[];
			[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           	anguloActual=pasarAGrados(anguloRads);
           	i=1;
           	while i < 13
           		turnAngle(serPort, .2,30);
           		distancia(i)= ReadSonarMultiple(serPort,1)
				[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           		angulos(i)=adaptarGrados(pasarAGrados(anguloRads));
           		i=i+1;
				pause(0.1);
			end
			indice=find(distancia==min(distancia));
           	[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           	anguloSensor=pasarAGrados(anguloRads);
           	anguloDestino= angulos(indice);
           	anguloSensor= adaptarGrados(anguloSensor);
           	if(anguloDestino > anguloSensor)
           		anguloGiro = valorAbsoluto(anguloDestino-anguloSensor);
           		turnAngle(serPort, .2,anguloGiro);		
           	else
           		anguloGiro = valorAbsoluto(anguloDestino-anguloSensor);
           		turnAngle(serPort, .2,-anguloGiro);
         	end    				
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
				if distDerecha < 0.3 || distIzquierda < 0.3  || distFrontal < 0.3 
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
				convertido=180 +valor
				convertido=180+convertido;
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

