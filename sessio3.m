%////////////////////
% sessio3.m
%////////////////////

function sessio3(serPort)
		global qGoal;
		bug1(serPort,[-5,-4]);
		
		
		function bug1(serPort,objectiu)
			
			while true
				qGoal=objectiu;
				obstacle=false;
				[BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = ...     
	                 BumpsWheelDropsSensorsRoomba(serPort);
	            [x, y, anguloRads]=OverheadLocalizationCreate(serPort);
				DecisionAnguloGiro(x, y, anguloRads,objectiu);
				while ~hayObstaculo() && ~hemArribat([x, y], objectiu)
					fprintf('Entramos en Bucle principal');
					[x, y]=OverheadLocalizationCreate(serPort);
					 SetDriveWheelsCreate(serPort,.5,.5);
					 pause(0.000001);
				end 
				if hayObstaculo()
					SetDriveWheelsCreate(serPort,.0,.0);

				end
				if hemArribat([x, y], objectiu)
					fprintf('hemos llegado a la meta , bien!!!!');
					return;
				end
				[x, y]=OverheadLocalizationCreate(serPort);
				puntoInicialObstaculo=[x, y]
				
				followBoundary(serPort,objectiu,puntoInicialObstaculo);
				
					desPegarmeDeObjeto();
					[x, y,anguloRads]=OverheadLocalizationCreate(serPort);
					DecisionAnguloGiro(x, y, anguloRads,objectiu);
					beep();
					return;
					while ~hayObstaculo() && ~hemArribat([x, y], objectiu)
						[x, y]=OverheadLocalizationCreate(serPort);
					 	SetDriveWheelsCreate(serPort,.5,.5);
					 	
					end
					if hemArribat([x, y], objectiu)
						fprintf('hemos llegado a la meta , bien!!!!');
						return;
					end
					if hayObstaculo()
						SetDriveWheelsCreate(serPort,.0,.0);
						
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
           		distancia(i)= ReadSonarMultiple(serPort,1)
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
				if vueltaCompleta(x_actual, y_actual,puntoInicialObstaculo) && contador > 300
					%quiere decir que hemos dado una vuelta ahora miraremos las distancias
					%que hemos ido guardando para cuando estemos muy cercanos a la mejor ir hacia el 
					%objetivo
					hemosDadoVuelta=true;
				end
				pause(0.0001);
				distanciaDerecha=ReadSonarMultiple(serPort,1);
				distanciaFrontal=ReadSonarMultiple(serPort,2);
				%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
				if hemosDadoVuelta
					distanciaActual=[];
					distanciaActual(1)=x_actual;
					distanciaActual(2)=y_actual;
					dist=getDistancia(distanciaActual,qGoal);
					vectDistancias
					if puntoMasCercanRodeado(dist,vectDistancias)
						fprintf('podemos dejar de rodear el objeto\n');
						
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
						%%%%% asignamos distancia al punto y posicion
						
						
						%vectPosicionX(i)=x_actual;
						%vectPosicionY(i)=y_actual;
						contador=contador+1;
						%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					if distanciaFrontal < 0.3
						SetDriveWheelsCreate(serPort,.0,.0);
						while true
							turnAngle(serPort, .2,1);
							distanciaFrontal=ReadSonarMultiple(serPort,2);

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
		
		function retorno= vueltaCompleta(x_actual, y_actual,puntoInicialObstaculo)
		% Nos avisa que hemos rodeado por completo el obstáculo.
		% Entrada (x,y) y nos devuelve boolean.
		% True si puntoInicialObstaculo=[x_actual, y_actual] con un margen de ~0.30.

		dx = valorAbsoluto(x_actual)-valorAbsoluto(puntoInicialObstaculo(1));
		dy = valorAbsoluto(y_actual)-valorAbsoluto(puntoInicialObstaculo(2));
		retorno = false;
		
			if valorAbsoluto(dx) <= 0.3 && valorAbsoluto(dy) <= 0.3
				retorno = true;
			else
				retorno = false;
			end

		end
		function desPegarmeDeObjeto()
			%esta funcion lo que hace es despegarse del objeto 
			%una vez hemos detectado que el sensor derecho nos 
			%devuelve una distancia mas grande que 0.8 
			%beep();
			i=1;
           	while i < 13 
           		[x, y, anguloRads]=OverheadLocalizationCreate(serPort);
           		anguloActual=pasarAGrados(anguloRads);
				distDerecha= ReadSonarMultiple(serPort,1);
				if distDerecha > 0.8
					travelDist(serPort,0.1,0.4);
					return;
				end
           		turnAngle(serPort, .2,30); 
           		i=i+1;
				pause(0.1);
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
						   )

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
		function DecisionAnguloGiro(x, y, anguloRads,objectiu)
			% x , y->>>> posicion actual
			% esta funcion decide hace donde girar el robot para que haga 
			% el minimo giro posible y ademas se mueva hacia el angulo que 
			% forma la linia mas corta hacia el objetivo atan(angulo)
			% donde anguloRads es el angulo actual del robot en sentio antihorario
			% donde objectiu es el punto x e y del punto final al que llegar
			% donde angulo giro sera el angulo al que girara el robot hacia el objetivo
			anguloGiro=0
			catetoContiguo=0	% para calcular el angulo
			catetoOpuesto=0		% para calcular el angulo

			anguloActual=pasarAGrados(anguloRads);
			if objectiu(1) >= 0 && objectiu(2) >= 0		% primer cuadrante	
				catetoContiguo=objectiu(1)-x;
				catetoOpuesto=objectiu(2)-y;
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro);
				anguloGiro=abs(anguloGiro)
				turnAngle(serPort, .2,anguloGiro);
			elseif objectiu(1) < 0 && objectiu(2) >= 0	% segundo cuadrante
				catetoContiguo=abs(objectiu(1)-x);
				catetoOpuesto=abs(objectiu(2)-y);
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro);
				anguloGiro=abs(180-anguloGiro)
				turnAngle(serPort, .2,anguloGiro);
			elseif objectiu(1)<0 && objectiu(2) < 0 	% tercer cuadrande 
				catetoContiguo=abs(objectiu(1)-x);
				catetoOpuesto=abs(objectiu(2)-y);
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro)
				anguloGiro=abs(180+anguloGiro)
				turnAngle(serPort, .2,anguloGiro);
			else
				pause(2);					% cuarto cuadrande
				catetoContiguo=abs(objectiu(1)-x);
				catetoOpuesto=abs(objectiu(2)-y);
				anguloGiro=atan(catetoOpuesto/catetoContiguo);
				anguloGiro=pasarAGrados(anguloGiro)
				anguloGiro=abs(anguloGiro)

				turnAngle(serPort, .2,-anguloGiro);
			end


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
