%////////////////////
% sessio3.m
%////////////////////

function bug2(serPort)
		global qGoal;
		global puntInicial;
		[xTarget, yTarget]=ginput(1);
		bug2(serPort,[xTarget,yTarget]);
			
		
		function bug2(serPort,objectiu)

			while true
				pause(0.001);
				clc;
				display('iniciamos proceso');
				qGoal=objectiu;
				obstacle=false;
				[BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...     
		            BumpsWheelDropsSensorsRoomba(serPort);
				[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
				puntInicial=[x,y];
				DecisionAnguloGiro(objectiu);
				while ~hayObstaculo() && ~hemArribat([x, y], objectiu)
					display('Entramos en Bucle principal');
					[x, y]=OverheadLocalizationCreate(serPort);
					SetDriveWheelsCreate(serPort,.5,.5);
					pause(0.000001);
				end 
				if hayObstaculo()
					SetDriveWheelsCreate(serPort,.0,.0);

				end
				if hemArribat([x, y], objectiu)
					display('hemos llegado a la meta , bien!!!!');
					return;
				end
				[x, y]=OverheadLocalizationCreate(serPort);
				puntoInicialObstaculo=[x, y]
				followBoundary(serPort,objectiu,puntoInicialObstaculo);
				while true
					pause(0.001);
					SetDriveWheelsCreate(serPort,.1,.1);
					distFrontal = ReadSonarMultiple(serPort,2);
					[x, y]=OverheadLocalizationCreate(serPort);
					if distFrontal < 0.2
						%paramos i seguimos con lo de antes
						break;
					end
					if hemArribat([x, y], objectiu)
						display('hemos llegado a la meta , bien!!!!');
						return;
					end

				end
			end

		end
		function preFollowBoundary()
			% Una vez nos encontramos un obstaculo lo dejaremos a la derecha del robot.
			% Para ello damos una vuelta de 360º sobre nuestro eje y vamos anotando los valores del sensor derecho
			% en un vector. Una vez que tenemos el valor más pequeño giramos el ángulo que corresponde.

			distancia=[];
			angulos=[];
			[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           	anguloActual=pasarAGrados(anguloRads);
           	i=1;
           	while i < 13 
           		turnAngle(serPort, .2,30); 
           		distancia(i)= ReadSonarMultiple(serPort,1);
				[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           		angulos(i)=adaptarGrados(pasarAGrados(anguloRads));
           		i=i+1;
				pause(0.1);
			end
			indice=find(distancia==min(distancia));
           	anguloDestino= angulos(indice);
           	anguloSensor= getAnguloActual();
           	if(anguloDestino > anguloSensor)
           		anguloGiro = valorAbsoluto(anguloDestino-anguloSensor);
           		turnAngle(serPort, .2,anguloGiro);		
           	else
           		anguloGiro = valorAbsoluto(anguloDestino-anguloSensor);
           		turnAngle(serPort, .2,-anguloGiro);
         	end    				
		end
		function condicion=abandonarObjeto(contador)
				[x_actual, y_actual]=OverheadLocalizationCreate(serPort);
				condicion=false;
				if contador > 1000
					contador
						if eqContinua (puntInicial, qGoal, [x_actual, y_actual])
							beep();
							DecisionAnguloGiro(qGoal);
							condicio=true;
							return;
						end
				end
		end	
		function followBoundary(serPort,objectiu,puntoInicialObstaculo)
			preFollowBoundary();%nos posicionamos para que el obstaculo quede a nuestra derecha
			vectDistancias=[];
			vectPosicionX=[];
			vectPosicionY=[];
			distanciaDerecha=ReadSonarMultiple(serPort,1);
			contador=1;
			indice=1;
			hemosDadoVuelta=false;
			while true
				[x_actual, y_actual]=OverheadLocalizationCreate(serPort);
				posicionActual=[];
				posicionActual(1)=x_actual;
				posicionActual(2)=y_actual;
				vectDistancias(indice)=getDistancia(posicionActual,qGoal);
				indice=indice+1;
		
				distanciaDerecha=ReadSonarMultiple(serPort,1);
				distanciaFrontal=ReadSonarMultiple(serPort,2);
				%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
				if contador > 1000
					contador
						if eqContinua (puntInicial, qGoal, [x_actual, y_actual])
							beep();
							DecisionAnguloGiro(qGoal);
							condicio=true;
							return;
						end
				end
				%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
				if distanciaDerecha < 0.20
					SetDriveWheelsCreate(serPort,.0,.0);
					turnAngle(serPort, .2,1);
				end
				%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
				if distanciaDerecha >= 0.20 && distanciaDerecha <= 0.40
						SetDriveWheelsCreate(serPort,.1,.1);
						contador=contador+1;
						%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					if distanciaFrontal < 0.3
						SetDriveWheelsCreate(serPort,.0,.0);
						while true
							turnAngle(serPort, .2,1);
							distanciaFrontal=ReadSonarMultiple(serPort,2);
							if contador > 1000
								contador
									if eqContinua (puntInicial, qGoal, [x_actual, y_actual])
										beep();
										DecisionAnguloGiro(qGoal);
										condicio=true;
										return;
									end
							end
							if distanciaFrontal > 2.5 && distanciaFrontal <= 3
								break;
							end
						end	
					end

				end
				%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%				
				if distanciaDerecha > 0.4
					SetDriveWheelsCreate(serPort,.0,.0);
					pause(2);
					turnAngle(serPort, .2,-15);
					if distanciaDerecha > 0.4
						travelDist(serPort,0.1,0.1);
					end

				end
				%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
			end           
		end	
		function  condicion= puntoMasCercanRodeado(distanciaActual,vectorDistancias)
			%esta funcion nos mira la diferencia entre la distancia actual al qGoal
			% y las distancias guardadas en el vectorDistancias
			%si la resta de la actual con alguna de todas las que hay en el vector 
			%es menor que ~0.5 ese es nuestro margen quiere decir que podemos dejar de rodear
			%el objeto
			condicion=false;
			valorMinimo=min(vectorDistancias);
			diferencial=valorAbsoluto(distanciaActual-valorMinimo);
			if diferencial <= 0.5
					condicion=true;
					return;
			end
		end
		function anguloGiro=getGiroDesdeAngulos(anguloInicial,anguloActual)
			% esta funcion lo que hace es calcular el angulo de giro 
			% que tiene que hacer dados dos angulos ejemplo
			% anguloInicial 180 anguloActual 190 retornaria 10
			if (anguloInicial < anguloActual)
				anguloActual=  anguloActual - anguloInicial ; 
				anguloGiro=-anguloActual;
			else
				anguloGiro = anguloInicial - anguloActual; 
			end	
		end
		function anguloActual=getAnguloActual()
			% esta funcion nos devuelve el angulo actual del robot pasado a 
			% grados i ademas con intervalo de entre 0 i 360
			[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           	anguloCorrecto=pasarAGrados(anguloRads);
			anguloActual=adaptarGrados(anguloCorrecto);
		end
		function distancia=getDistancia(puntoA,puntoB)
			%importante puntoA y puntoB son un vector de 2 coordenadas
			distancia=sqrt( ((puntoA(1)-puntoB(1)).^2)...
						    + ((puntoA(2)-puntoB(2)).^2)...
						   );

		end
		function distancia=hemArribat(posicioActual,objectiu)
			
			 distancia=sqrt( ((objectiu(1)-posicioActual(1)).^2)...
						    + ((objectiu(2)-posicioActual(2)).^2)...
						   );
			 % distancia;
			 if distancia < 0.5
			 	distancia=true;
			 else 
			 	distancia=false;
			 end
		end
		
		function trobat=hayObstaculo()
				 trobat=false;
				 distDerecha= ReadSonarMultiple(serPort,1);
           		 distFrontal = ReadSonarMultiple(serPort,2);
           		 distIzquierda = ReadSonarMultiple(serPort,3);
				if distDerecha < 0.5 || distIzquierda < 0.5  || distFrontal < 0.5 
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
				valor=-valor;
			else
				valor=valor;
			end
		end

		
		function grados=pasarAGrados(angulo)
			angulo=double(angulo);
			grados=double(angulo*(180/pi));
		end
		function DecisionAnguloGiro(objectiu)
			
				
				qGoal=objectiu;
				[x1, y1,angulo]=OverheadLocalizationCreate(serPort);
				angulo=pasarAGrados(angulo);
				angulo=adaptarGrados(angulo);

				if angulo <=  180
					turnAngle(serPort, .2,-angulo);
				else
					angulo=360-angulo;
					turnAngle(serPort,.2,angulo);
				end
				[x1, y1,angulo]=OverheadLocalizationCreate(serPort);
				x2=objectiu(1);
				y2=objectiu(2);
				dx=valorAbsoluto(x1-x2);
				dy=valorAbsoluto(y1-y2);
				angulo=atan(dy/dx);
				anguloGiro=pasarAGrados(angulo);
				if x2 >= x1 && y2 >= y1
					turnAngle(serPort, .2,anguloGiro);
					
				elseif x2 <= x1 && y2 >= y1
					turnAngle(serPort, .2,180-anguloGiro);
					anguloExacto=(180-anguloGiro)
					
				elseif x2 <=x1 && y2 <= y1
					angulo=valorAbsoluto(anguloGiro-90);
					angulo=angulo+90;
					turnAngle(serPort, .2,-angulo);
					
			    elseif x2 >= x1 && y2 <= y1
			    	anguloGiro
			    	angulo=valorAbsoluto(anguloGiro);
			    	turnAngle(serPort, .2,-anguloGiro);
			    	beep();
			    end

				[x1, y1,angulo]=OverheadLocalizationCreate(serPort);
				anguloExacto=anguloGiro+180;
				anguloActual=pasarAGrados(angulo);
				anguloNoExacto=adaptarGrados(anguloActual);
				puntoA=[x1, y1]
				puntoB = objectiu;

		end
		function convertido=adaptarGrados(valor)
			% esta funcion coge el valor i si es negativo lo 
			% adapta para que el valor este en un intervalo de 
			% 0 a 360 no de 0 a 180 i -180 a 0
			if valor < 0
				convertido=180 +valor
				convertido=180+convertido;
			else
				convertido=valor;
			end
		end
		function condicion = eqContinua (puntoA, puntoB, pertenece)
			condicion=false;
			vector=puntoB-puntoA;
			
			izqRes=((pertenece(1)-puntoA(1))/vector(1))
			derRes=((pertenece(2)-puntoA(2))/vector(2))
			error=valorAbsoluto(izqRes-derRes)
			if error < 0.2
				display('El punto pertenece a la recta ')
				condicion=true;
				return;
			else
				display('El punto NO pertenece a la recta ')
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
