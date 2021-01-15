#include <sstream>
#include <string>
#include <fstream>
#include <math.h>
#include "Graph.h"
//Euklidischer Abstand Zweier Knoten
double distance(int a, int b, double array[][2]){
  return sqrt((array[a][0]-array[b][0])*(array[a][0]-array[b][0])
  +(array[a][1]-array[b][1])*(array[a][1]-array[b][1]));
}


//Fill Arrays from txt files
graph answer(){
  graph x; //x returns our two Arrays

  int anzahl_knoten = 42;
  int anzahl_kanten = 72;
  //Array von Knotenkoordinaten (Knotennummer X Position)
  double koordinaten[anzahl_knoten][2];
  //Array von Kanten (Kantennummer X Kante A, Kante B, Distance)
  double kanten[anzahl_kanten][3];
 //Init Arrays with Zero
  for(int i=0; i<anzahl_knoten; i++){
    koordinaten[i][0]=0;
    koordinaten[i][1]=0;
  }
  for(int i=0; i<anzahl_kanten; i++){
    kanten[i][0]=0;
    kanten[i][1]=0;
    kanten[i][2]=0;
  }
//Fill Array from txt file
  std::ifstream infile_knoten("knoten_prakt_01.txt");
  std::string line_knoten;
  while (std::getline(infile_knoten, line_knoten))
  {
      std::istringstream iss(line_knoten);
      int a;
      double b, c;
      if (!(iss >> a >> b >> c)) { break; } // error
      koordinaten[a][0]=b;
      koordinaten[a][1]=c;
  }

  std::ifstream infile_kanten("kanten_prakt_01.txt");
  std::string line_kanten;
  while (std::getline(infile_kanten, line_kanten))
  {
    std::istringstream iss(line_kanten);
    int a, b, c;
    if (!(iss >> a >> b >> c)) { break; } // error
    kanten[a][0]=b;
    kanten[a][1]=c;
    kanten[a][2]=distance(b,c, koordinaten);
  }

  for(int i=0; i<=41; i++){
    for(int j=0; j<=1; j++){
       x.knotengraph[i][j] = koordinaten[i][j];
    }
  }
  for(int i=0; i<=71; i++){
    for(int j=0; j<=2; j++){
      x.kantengraph[i][j] = kanten[i][j];
    }
  }
  return x;
}





/*  for(int i=0; i<anzahl_knoten; i++){
  printf("%f , %f \n", koordinaten[i][0], koordinaten[i][1]);
}
printf("Look Ma I made it\n");

for(int i=0; i<anzahl_kanten; i++){
  printf("%f , %f, %f \n", kanten[i][0], kanten[i][1], kanten[i][2]);
}
printf("Here we are now. Entertain us\n");
*/
