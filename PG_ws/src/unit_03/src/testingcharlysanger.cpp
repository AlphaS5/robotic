#include <sstream>
#include <string>
#include <fstream>
#include <math.h>
//#include "Graph.h"
#include "Graph.cpp"

int main(){

  graph ourx = answer();

  double knoten[42][2];
  for(int i=0; i<=41; i++){
    for(int j=0; j<=1; j++){
      knoten[i][j] = ourx.knotengraph[i][j];
    }
  }

  double kanten[72][3];
  for(int i=0; i<=71; i++){
    for(int j=0; j<=2; j++){
      kanten[i][j] = ourx.kantengraph[i][j];
    }
  }

  for(int i=0; i<42; i++){
    printf("%f , %f \n", knoten[i][0], knoten[i][1]);
    }
    printf("Look Ma I made it\n");

  for(int i=0; i<72; i++){
    printf("%f , %f, %f \n", kanten[i][0], kanten[i][1], kanten[i][2]);
    }
    printf("Here we are now. Entertain us\n");
return 0;
}
