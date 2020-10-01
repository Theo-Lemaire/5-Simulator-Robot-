%*****     NETTOYAGE DE LA FENETRE     *****
clc;
close all;
clear all;

global Nmin Cible R L Longueur wmax dmax dt angle_radar;
    
%*****     CAPTURE DE LA CIBLE ET POSITION INITIALE DU ROBOT     *****
E = [-12, 12];
C = [15, -5];

Etat = [E(1); E(2); 0; 0; 0];   % position initiale du robot [x; y; theta; wd; wg]
Cible = [C(1); C(2); 0];        % position de la cible à atteindre [xd; yd; thetad]



%*****     PARAMETRES DU ROBOT ET DE SIMULATION    *****
R = 0.05;   % rayon des roues
L = 0.4;    % 1/2 entraxe
Longueur = 0.8;
dt = 0.1;
commande = [0; 0];  % [wd,wg]
dmax = 3; % distance max capteur
wmax = 10;

angle_radar = pi/6;

Nmin = 0.25; % intervalle de distance pour l'arrivée

%i = 0;  % Timeout pour test
%Timeout = 1000;

%*****     BOUCLE PRINCIPALE    *****

while ((abs(Etat(1)-Cible(1)) > Nmin || abs(Etat(2)-Cible(2)) > Nmin)) %&& i < Timeout)
[Etat,Capteurs] = simulateur(Etat,commande);
commande = controle(Etat,Cible,Capteurs);
%i = i + 1   % Timeout pour test
end 
