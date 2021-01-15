#include <sstream>
#include <string>
#include <fstream>

int main(){
  //Array von Knotenkoordinaten (Knotennummer X Position)
  double koordinaten[42][2];
  for(int i=0; i<=41; i++){
    koordinaten[i][0]=0;
    koordinaten[i][1]=0;
  }
  std::ifstream infile("knoten_prakt_01.txt");

  std::string line;
  while (std::getline(infile, line))
  {
      std::istringstream iss(line);
      int a, b, c;
      if (!(iss >> a >> b >> c)) { break; } // error
      koordinaten[a][0]=b;
      koordinaten[a][1]=c;
  }
  //Euklidischer Abstand Zweier Knoten
  double distance(int a, int b){
    return sqrt((koordinaten[a][0]-koordinaten[b][0])*(koordinaten[a][0]-koordinaten[b][0])
    +(koordinaten[a][1]-koordinaten[b][1])*(koordinaten[a][1]-koordinaten[b][1]));
  }
  //Array von Kanten (Kantennummer X Kante A, Kante B, Distance)
  double kanten[72][3];
  for(int i=0; i<=71; i++){
    kanten[i][0]=0;
    kanten[i][1]=0;
    kanten[i][2]=0;
  }
  //Fill Array from txt file
  std::ifstream infile("kanten_prakt_01.txt");

  std::string line;
  while (std::getline(infile, line))
  {
    std::istringstream iss(line);
    int a, b, c;
    if (!(iss >> a >> b >> c)) { break; } // error
    kanten[a][0]=b;
    kanten[a][1]=c;
    kanten[a][2]=distance(b,c);
  }

  for(int i=0; i<=41; i++){
  printf("%f , %f \n", koordinaten[i][0], koordinaten[i][1];
  }
  printf("Look Ma I made it");

  for(int i=0; i<=71; i++){
  printf("%f , %f, %f \n", kanten[i][0], kanten[i][1], kanten[i][2]);
  }
  printf("Here we are now. Entertain us");
  return 0;
  }

}
