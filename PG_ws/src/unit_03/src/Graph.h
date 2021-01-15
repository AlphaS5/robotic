

//Struct um 2 Arrays gleichzeitig zu returnen
struct graph{
    double knotengraph[42][2];
    double kantengraph[72][3];
};
typedef struct graph graph;

/*
HOW TO ACCESS OUR DATA
graph x = answer();
double knoten = x.knotengraph;
*/

graph answer();
