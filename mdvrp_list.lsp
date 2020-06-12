/********** mdvrp_new.lsp **********/

use io;

/* Reads instance data
 * The input files follow the "Cordeau" format.*/
function input() {
    usage = "\nUsage: localsolver mdvrp_new.lsp inFileName=inputFile" + "\n[lsTimeLimit=timeLimit] "
                                                                      + "\n[lsTimeBetweenDisplays=timeBetweenDisplays] "
                                                                      + "\n[qgisOutputFileName=outputFile1] "
                                                                      + "\n[solOutputFileName=outputFile2] \n";
    if (inFileName == nil) throw usage;

    readInputMDVRP();
    computeDistanceMatrix();
}


/* Declares the optimization model. */
function model() {
    // Sequence of customers visited by each truck.
    customersSequences[k in 1..nbTrucks] <- list(nbCustomers);
    // All customers must be visited by  the trucks
    constraint partition[k in 1..nbTrucks](customersSequences[k]);

    for [k in 1..nbTrucks] {
        local sequence <- customersSequences[k];
        local c <- count(sequence);

        // A truck is used if it visits at least one customer
        trucksUsed[k] <- c > 0;

        // The quantity needed in each route must not exceed the truck capacity
        routeQuantity[k] <- sum(0..c-1, i => demands[sequence[i]]);
        constraint routeQuantity[k] <= truckCapacity;
            //overload[k] <- max(0, routeQuantity - truckCapacity);

        // Route distance without starting and stoping at depot
        routeDistanceWithOutDepot[k] <-  trucksUsed[k] ? distanceMatrix[sequence[0]][sequence[c - 1]] + sum(1..c-1, i => distanceMatrix[sequence[i - 1]][sequence[i]]): 0;
        // Distance travelled by truck k starting at depot d
        firstAndLastDistance[k] <- trucksUsed[k] ? (c > 1 ? min[d in 0..nbDepot-1](min(0..c-1, i => distanceWarehouse[d][sequence[i]]
                                                                  + distanceWarehouse[d][sequence[(i+1)%c]]
                                                                  - distanceMatrix[sequence[i]][sequence[(i+1)%c]]))
                                                          : min[d in 0..nbDepot-1](2 * distanceWarehouse[d][sequence[0]]))
                                                 : 0;
        routeDistances[k] <- routeDistanceWithOutDepot[k] + firstAndLastDistance[k];

        if (routeDistanceCapacity>0){
            constraint routeDistances[k] <= routeDistanceCapacity;
        }
    }

    // Number of trucks used
    nbTrucksUsed <- sum[k in 1..nbTrucks](trucksUsed[k]);

    // Total distance travelled
    totalDistance <- sum[k in 1..nbTrucks](routeDistances[k]);
    // Objective: minimize the distance travelled, then minimize the number of trucks used
    minimize totalDistance;
    //minimize nbTrucksUsed;

}

/* Parameterizes the solver. */
function param() {
    if (lsTimeLimit == nil) lsTimeLimit = 10;
    if (lsNbThreads == nil) lsNbThreads = 2;
    if (lsTimeBetweenDisplays == nil) lsTimeBetweenDisplays = 1;
        
    chosenModel = "expert model";
    
    instancePath = inFileName.split("/");
    instanceName = instancePath[instancePath.count()-1];
    
    if (qgisOutputFileName == nil)
        qgisOutputFileName = "qgis_files/solutions/" + chosenModel + "/sol_" + instanceName + "int_dist.csv";
    if (solOutputFileName == nil)
        solOutputFileName = "sol_files/" + chosenModel + "/" + instanceName + "int_dist.res";
}



function output() {
    println("totalDistance: ", totalDistance.value);
    println("nbTrucksUsed: ", nbTrucksUsed.value);
}


function readInputMDVRP() {
    // Input files following "Cordeau"'s format
    local inFile = io.openRead(inFileName);
    local dump = inFile.readInt();
    nbTrucksForEachDepot = inFile.readInt();
    nbCustomers = inFile.readInt();
    nbDepot = inFile.readInt();
    nbTrucks = nbDepot*nbTrucksForEachDepot;
    routeDistanceCapacity = inFile.readInt();
    truckCapacity = inFile.readInt();
    //println("nbTrucks:",nbTrucks, ", nbCustomers:",nbCustomers," ,nbDepot:",nbDepot," ,truckCapacity:",truckCapacity," ,DistanceCapacity:",routeDistanceCapacity);
    for[i in 1..nbDepot-1] dump = readln(inFile);

    // Start parsing of the customers data :
    for[n in 0..nbCustomers-1] {
      inLine = readln(inFile);
      line = inLine.split();
      if(count(line) == 0) break;
      if(count(line) < 5) error("Wrong file format");
      nodesX[n] = round(toDouble(line[1]));
      nodesY[n] = round(toDouble(line[2]));
      demands[n] = toInt(line[4]);
    }
    for[n in 0..nbDepot-1]{
      inLine = readln(inFile);
      line = inLine.split();
      DepotX[n] = round(toDouble(line[1]));
      DepotY[n] = round(toDouble(line[2]));
    }
}


/* Compute the distance between each node */
function computeDistanceMatrix() {
    for[i in 0..nbCustomers-1]
    {
        distanceMatrix[i][i] = 0;
        for[j in i+1..nbCustomers-1]
        {
            local localDistance = computeDist(i, j);
            distanceMatrix[j][i] = localDistance;
            distanceMatrix[i][j] = localDistance;
        }
    }

    for[i in 0..nbCustomers-1] {
      for[d in 0..nbDepot-1]{
        local localDistance = computeDistDepot(d, i);
        distanceWarehouse[d][i] = localDistance;
      }
    }
}

function computeDist(i,j) {
    local exactDist = sqrt(pow((nodesX[i] - nodesX[j]), 2) + pow((nodesY[i] - nodesY[j]), 2));
    return round(exactDist);
}

function computeDistDepot(d,j) {
    local exactDist = sqrt(pow((DepotX[d] - nodesX[j]), 2) + pow((DepotY[d] - nodesY[j]), 2));
    return round(exactDist);
}