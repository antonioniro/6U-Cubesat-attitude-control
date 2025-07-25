%sintassi funzione di richiamo per l'algoritmo Quest
function q=QUEST(V,W,a)

K=Gain_matrix(V',W',a);


[A,D]=eig(K);%fornisco max autovalore e indici del massimo fornendoli in un vettore
                         %risoluzione del problema degli autovalori

[G,Imax]=max(real(diag(D)));%prendo elementi della matrice degli autovettori i cui indici sono 
                      %quelli corrispondenti al max autovalore


q=A(:,Imax);
return
