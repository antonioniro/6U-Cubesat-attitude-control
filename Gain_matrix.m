%sintassi per arrivare a matrice K che compare in gain function qualunque
%sia il num di vettori in input
function K=Gain_matrix(V,W,a)

[~,cv]=size(V);
%clear rv    %in questo modo ho una sommatoria di termini il cui num 
% è uguale al numero di colonne della matrice in cui scrivo vettori
% di osservazione

B = zeros(3,3);

for i=1:1:cv
    B = B + a(i)*W(:,i)*(V(:,i))';
end

sigma=0;
for i=1:1:cv
    sigma= sigma + a(i)*dot(W(:,i),(V(:,i))');
end

Z = zeros(3,1);
for i=1:1:cv
    Z= Z + a(i)*cross(W(:,i),V(:,i));
end

S=B+B';
%Z=B-B'
I=eye(3);
L=sigma*I;
F=S-L; % è la S-sigmaI
R=Z';

K = zeros(4,4);
K=[F Z(:); Z(:)' sigma];     %metto Z(:) poichè così il vettore
%è automaticamente adattato a righe o colonne

return
