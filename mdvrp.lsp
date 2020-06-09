/********** mdvrp.lsp **********/
/* A generalized multi-depot vehicle routing problem with replenishment based on LocalSolver /
Int. J. Industr. Engg. Comput., 2015, vol. 6, 81 - 98
*/

use io;
use math;

/* The input files follow the "Solomon" format.*/
function input() {
    usage = "\nUsage: localsolver cvrptw.lsp "
    + "inFileName=inputFile [solFileName=outputFile] [lsTimeLimit=timeLimit]\n";

    if (inFileName == nil) throw usage;

    readInputmdvrp();
    println("@@ Data reading completed ...");
    println(".. nbCustomers: ", nbCustomers);
    println(".. nbWarehouse: ", nbWarehouse);
    println(".. nbTrucks: ", nbTrucks);

    // Compute distance matrix
    computeDistanceMatrix();
    println("@@ Distance matrix computed ...");

}

/* Declares the optimization model. */
function model() {

    // Define the design variable
    N0[0] = 0; N0m = map(); name_dict = map();
    name_dict[0] = "0";
    for [i in 1..nbCustomers]{
        N0[i] = i;
        N0m[customerName[i-1]] = i;
        name_dict[i] = customerName[i-1];
    }
    for [j in 1..nbWarehouse]{
        N0[i+j] = i+j;
        N0m[warehouseName[j-1]] = i+j;
        name_dict[i+j] = warehouseName[j-1];
    }
    // println(N0, N0.count());
    // println(name_dict);

    s = 1 + nbCustomers + nbWarehouse;
    N = nbCustomers + nbWarehouse;
    nc = N0.count();
    // println(nc);

    // Vehicle-Customer Relation
    // already populated while reading the data
    // println(customerVehicleChoice);

    //for [k in 0..nbTrucks-1]{
    //    println(k, truckTerminal[k], N0m[truckTerminal[k]]);
    //}

    // demand Vector, preparation time vector
    q = {0}; prep = {0};
    for [i in 1..nc-1]{
        if (i >= 1 && i <= nbCustomers){
            q[i] = customerDemands[i-1];
            prep[i] = customerPrepTime[i-1];
        }
        if (i > nbCustomers) {
            q[i] = 0;
            prep[i] = warehousePrepTime[i-nbCustomers-1];
        }
    }


    // x_ink = 1 if node-i is assigned in sequence-n for vehicle-k
    // i ranges from 0 to |C|+|W|
    for [i in 0..nc-1][n in 0..s][k in 0..nbTrucks-1]{

        x[i][n][k] <- bool();
        // first and last position of the Trucks should be the same warehouse
        if (n == 0 || n == s){
            x[i][n][k] <- i==N0m[truckTerminal[k]];
        } 

        // for customers
        if (i >= 1 && i <= nbCustomers){
            permittedVehicles = customerVehicleChoice[i-1];
            // println(k, permittedVehicles, mapNotContains(permittedVehicles, k));
            // if vehicle-k is not in the customer's preference
            if (mapNotContains(permittedVehicles, k)){
                x[i][n][k] <- 0;
            }
        }
    }

    // need to maximize
    // Eq.(3)
    totalCustomersServed <- sum[i in 1..nbCustomers](customerPriority[i-1]*(sum[n in 0..s][k in 0..nbTrucks-1](x[i][n][k])));

    // intermediate variables
    // y_nk <- node visited at sequence-n by vehicle-k
    for [n in 0..s][k in 0..nbTrucks-1]{
        y[n][k] <- min(N, sum[i in 1..nc-1](i*x[i][n][k]));  // Eq.(1)
    }
    
    // z_nk <- location of vehicle-k at sequence-n
    for [k in 0..nbTrucks-1] z[0][k] <- N0m[truckTerminal[k]];  // initially all trucks are at their respective warehouse
    for [n in 1..s][k in 0..nbTrucks-1]{
        z[n][k] <- sum[i in 1..nc-1](x[i][n][k])==1 ? y[n][k] : z[n-1][k]; // Eq. (2)
    }

    // Fill in 1-D array of the distanceMatrix
    D = {};
    for [i in 0..nc-1][j in 0..nc-1]{
        D[i+nc*j] = distanceMatrix[i][j];
    }

    // create dis_nk variable
    for [k in 0..nbTrucks-1] {
        dis[0][k] <- 0;
    }
     for [n in 1..s][k in 0..nbTrucks-1]{
        dis[n][k] <- D[z[n][k] + nc*z[n-1][k]];    
    }

    // need to minimize
    // totalTravelDistance <- sum[k in 0..nbTrucks-1][n in 1..s] (D[z[n][k] + nc*z[n-1][k]]); // Eq. (6)
    totalTravelDistance <- sum[k in 0..nbTrucks-1][n in 0..s] (dis[n][k]);

    // Vehicle replenishment related
    for [n in 0..s][k in 0..nbTrucks-1]{
        q_change[n][k] <- y[n][k] > nbCustomers ? truckCapacity[k] : -q[y[n][k]];  // Eq.(7)
    }

    for [k in 0..nbTrucks-1] q_end[0][k] <- truckCapacity[k];  // initially all trucks are fully loaded    
    for [n in 1..s][k in 0..nbTrucks-1]{
        q_end[n][k] <- y[n][k] > nbCustomers ? truckCapacity[k] : q_end[n-1][k] + q_change[n][k];  // Eq.(8)
        q_start[n][k] <- y[n][k] > nbCustomers ? q_end[n-1][k] : 0;  // Eq.(9)
    }

    // need to minimize
    totalLoading <- sum[k in 0..nbTrucks-1][n in 1..s] (q_start[n][k]); // Eq. (10)

    // whether a vehicle is used or not
    for [k in 0..nbTrucks-1]{
        isUsed[k] <- sum[i in 1..nbCustomers][n in 1..s-1] (x[i][n][k]) > 0 ? 1 : 0;
    }
    totalVehicleUsed <- sum[k in 0..nbTrucks-1] (isUsed[k]);

    // total cost (need to minimize)
    // isUsed[k]*truckCost[k]
    // currently truckCost[k] is not provided
    totalCost <- sum[k in 0..nbTrucks-1] ( truckCostPerKM[k]*sum[n in 1..s](dis[n][k]) );  // Eq. (13)

    // minimize number of replenishments
    totalReplenishment <- sum[i in 1..nc-1][n in 1..s-1][k in 0..nbTrucks-1] (x[i][n][k]);

    // Constraints
    for [i in 1..nbCustomers] {
        customerServe[i] <- sum[k in 0..nbTrucks-1][n in 1..s-1] (x[i][n][k]);
        constraint customerServe[i] <= 1;  // Eq. (15)
    }

    for [k in 0..nbTrucks-1]{
        for [n in 1..s-1]{
            constraint sum[i in 0..nc-1](x[i][n][k]) <= 1; // Eq. (16), need to revisit
        }
    }

    // Warehouse-Customer Constraint
    for [k in 0..nbTrucks-1] {
        g[0][k] <- N0m[truckTerminal[k]];
    }
    for [k in 0..nbTrucks-1][n in 1..s]{
        g[n][k] <- y[n][k] > nbCustomers ? y[n][k] : g[n-1][k];
    }

    // Get all the warehouses that can provide orders for customer-i
    W = map();
    for [i in 0..nbCustomers-1] {
        W[i] = map();
        for [j in 0..nbWarehouse-1] {
            if (mapContains(warehouseProdType[j], customerProdType[i])) {
                W[i].add(j);
            }
        }
        // println(i, ", ", customerProdType[i], ", ", W[i]);
    }

    // build the compatible array for warehouse-customer Relation
    compatible = map();
    kount = 0;
    for [j in 0..nbWarehouse-1]{
        for [i in 0..nbCustomers-1]{
            if (mapContains(W[i], j)) {
                // println(i, " ", j, ": ", mapContains(W[i], j));
                compatible.add(1);
            } else {
                // println(i, " ", j, ": ", mapContains(W[i], j));
                compatible.add(0);
            }
        }
        for [i in nbCustomers..nbCustomers+nbWarehouse-1]{
            if (i - nbCustomers == j){
                compatible.add(1);
            } else {
                compatible.add(0);
            }
        }
    }

    /*
    kount = 0;
    for [j in 0..nbWarehouse-1]{
        for [i in 0..nbCustomers+nbWarehouse-1]{
            print(compatible[kount], " ");
            kount = kount + 1;
        }
        println();
    }
    */

    for [k in 0..nbTrucks-1][n in 1..s]{
        constraint compatible[z[n][k]-1+N*(g[n][k]-nbCustomers-1)] == 1;
    }

    // Vehicle capacity constraint
    for [k in 0..nbTrucks-1][n in 0..s]{
        constraint q_end[n][k] >= 0;
    }

    // Warehouse capacity constraint
    // commenting this improves the results
    for [i in nbCustomers+1..nbCustomers+nbWarehouse]{
        // constraint sum[k in 0..nbTrucks-1][n in 0..s-1](x[i][n][k]*q_change[n][k]) <= warehouseCapacity[i-nbCustomers-1];

    }

    // Time window constraint
    // for [i in 0..nc-1][n in 0..s][k in 0..nbTrucks-1]
    // for [n in 0..s][k in 0..nbTrucks-1]

    // Fill ET and LT for customers and warehouses
    ET[0] = 0; LT[0] = 0;
    a[0] = 0; b[0] = 0;
    for [i in 1..nc-1]{
        if (i <= nbCustomers){
            ET[i] = customerET[i-1];
            LT[i] = customerLT[i-1];
            a[i] = customera[i-1];
            b[i] = customerb[i-1];
        } else {
            ET[i] = warehouseET[i-nbCustomers-1];
            LT[i] = warehouseLT[i-nbCustomers-1];
            a[i] = warehousea[i-nbCustomers-1];
            b[i] = warehouseb[i-nbCustomers-1];
        }
        // println(i, ", ", ET[i], ", ", LT[i]);
        // println(i, ", ", a[i], ", ", b[i]);
    }


    for [n in 0..s][k in 0..nbTrucks-1]{
        preTime[n][k] <- (sum[i in 1..nc-1](x[i][n][k])==1) ? prep[y[n][k]] : 0; // Eq. (21)
        loadTime[n][k] <- abs(q_change[n][k])/(y[n][k] > nbCustomers ? warehouseRefillSpeed[y[n][k]-nbCustomers-1] : truckUnloadSpeed[k]);
    }

    for [k in 0..nbTrucks-1]{
        timeStart[0][k] <- 0;
        // timeEnd[-1][k] <- 0;
        timeEnd[0][k] <- 0;
    }

    for [n in 1..s][k in 0..nbTrucks-1]{
        lunchBegin[n][k] <- max(timeEnd[n-1][k]+60*dis[n][k]/truckSpeed[k], ET[y[n][k]]);
        lunchEnd[n][k] <- lunchBegin[n][k] + preTime[n][k] + loadTime[n][k];
        isFinish[n][k] <- lunchBegin[n][k] < b[y[n][k]] && lunchEnd[n][k] > a[y[n][k]];
        timeStart[n][k] <- isFinish[n][k] ? b[y[n][k]] : lunchBegin[n][k];
        timeEnd[n][k] <- timeStart[n][k] + preTime[n][k] + loadTime[n][k];
    }

    // Time Window constraint
    for [n in 1..s][k in 0..nbTrucks-1]{
        constraint timeEnd[n][k] <= LT[y[n][k]];
    }

    // Maximal Duration Constraint
    for [k in 0..nbTrucks-1]{
        // constraint timeEnd[s][k] - timeStart[1][k] <= truckDuration[k];
    }

    // Maximal driving time constraint - not coded

    // Additional Acceleration Strategy - not coded

    // All objectives
    maximize totalCustomersServed;  // Eq. (3)
    minimize totalVehicleUsed;      // Eq. (12)
    minimize totalTravelDistance;   // Eq. (6)
    minimize totalLoading;          // Eq. (10)
    // minimize totalCost;
    minimize totalReplenishment;    // Eq. (14)

}

function minToString(val) {
    local h = math.floor(val/60);
    local m = math.round(val - h*60);
    return toString(h)+":"+toString(m);
}


function roundf(x, digit) {
    local x2 = math.round(x*math.pow(10, digit));
    return x2/math.pow(10, digit);
}

/* Parameterizes the solver. */
function param() {
    if (lsTimeLimit == nil) lsTimeLimit = 20;
}

/* Writes the solution in a file with the following format:
   - number of trucks used and total distance
   - for each truck the nodes visited (omitting the start/end at the depot) */
function output() {
    if (solFileName == nil) return;
    local outfile = io.openWrite(solFileName);

    customer_dict = map();
    for [k in 0..nbTrucks-1] {
        outfile.println("V",k+1);
        outfile.println("sequence | q_change | distance | cum.distance | start | end");
        cum_dist = 0;
        for [n in 0..s] {
            if (y[n][k].value>0){
                // println(k, " ", n, " ", y[n][k].value);
                outfile.print("n: ", n, ": ");
                outfile.print(name_dict[y[n][k].value], "\t");
                outfile.print(q_change[n][k].value, "\t");
                outfile.print(roundf(dis[n][k].value, 2), "\t");
                cum_dist = cum_dist + dis[n][k].value;
                outfile.print(roundf(cum_dist, 2), "\t");
                outfile.print(minToString(timeStart[n][k].value), "\t");
                outfile.print(minToString(timeEnd[n][k].value), "\t");
                outfile.println();
                customer_dict[name_dict[y[n][k].value]] = map();
                customer_dict[name_dict[y[n][k].value]].add(minToString(timeStart[n][k].value));
                customer_dict[name_dict[y[n][k].value]].add(minToString(timeEnd[n][k].value));
                customer_dict[name_dict[y[n][k].value]].add(k);
            }
        }
        outfile.println();
    }

    outfile.println();
    outfile.println("Customer | serviceTime | lunchTime | Truck");
    for [k in 0..nbCustomers-1]{
        cust_name = customerName[k];
        outfile.print(cust_name, ": ");
        // outfile.print(customerET[k], " ", customerLT[k], " ");
        // outfile.print(customera[k], " ", customerb[k], " ");
        outfile.print(serviceTime[N0m[cust_name]-1], "\t");
        outfile.print(lunchTime[N0m[cust_name]-1], "\t");
        if (mapContains(customer_dict.keys(), cust_name)){
            outfile.print(customer_dict[cust_name][0], "\t", customer_dict[cust_name][1], "\t", customer_dict[cust_name][2]);
        } else {
            outfile.print("Not served");
        }
        outfile.println();
    }
    outfile.println();

    /*
    for [n in 0..s] {
        // outfile.println("n: ", n, ": ", y[n][0].value, " ");
        outfile.print("n: ", n, ": ");
        for [k in 0..nbTrucks-1] {
            // outfile.print(y[n][k].value, " ");
            outfile.print(name_dict[y[n][k].value], " ");
        }
        outfile.println();
    }*/
}

function readInputmdvrp() {
    local inFile = io.openRead(inFileName);
    skipLines(inFile, 3);

    //Warehouse data
    nbWarehouse = inFile.readInt();
    skipLines(inFile, 1);
    for [i in 0..nbWarehouse-1] {
        local line = inFile.readln().split();
        if (count(line) == 0) break;
        if (count(line) != 8) throw "Wrong warehouse data format";
        warehouseIndex[i] = i;
        warehouseName[i] = line[0];
        warehouseX[i] = line[1].toDouble();
        warehouseY[i] = line[2].toDouble();
        warehouseCapacity[i] = line[3].toInt();
        warehouseTimeWindow[i] = line[4];
        warehouseET[i] = convertToMins(line[4].split("-")[0]);
        warehouseLT[i] = convertToMins(line[4].split("-")[1]);
        warehousea[i] = warehouseLT[i]+1;
        warehouseb[i] = warehouseLT[i]+2;
        warehousePrepTime[i] = line[5].toInt();
        warehouseRefillSpeed[i] = line[6].toDouble();
        if (line[7].indexOf(",")==-1) {
            warehouseProdType[i] = {line[7]};
        } else {
            prods = line[7].split(",");
            warehouseProdType[i] = prods;
        }

        i = i + 1;
    }
    skipLines(inFile, 2);

    // Vehicle Data
    nbTrucks = inFile.readInt();
    skipLines(inFile, 1);
    truckName2Index = map();
    for [i in 0..nbTrucks-1] {
        local line = inFile.readln().split();
        if (count(line) == 0) break;
        if (count(line) != 9) throw "Wrong Vehicle data format";
        truckIndex[i] = i;
        truckName[i] = line[0];
        truckName2Index[truckName[i]] = i;
        truckUnloadSpeed[i] = line[1].toDouble();
        truckCostPerKM[i] = line[2].toDouble();
        truckSpeed[i] = line[3].toInt();
        truckCapacity[i] = line[4].toInt();
        truckTimeWindow[i] = line[5];
        truckET[i] = convertToMins(line[5].split("-")[0]);
        truckLT[i] = convertToMins(line[5].split("-")[1]);
        truckMaxDistance[i] = line[6].toInt();
        truckDuration[i] = line[7].toInt()*60;  // convert to minutes
        truckTerminal[i] = line[8];
    }
    skipLines(inFile, 3);
    all_trucks = truckName2Index.values();

    //Customers data
    i = 0;
    while (!inFile.eof()) {
        inLine = inFile.readln();
        if (inLine.startsWith("//") || inLine.startsWith("**")) continue;
        line = inLine.split();
        // println(i, line);
        if (count(line) == 0) break;
        if (count(line) != 10) throw "Wrong Customer data format";
        customerIndex[i] = i; 
        customerName[i] = line[0];
        customerX[i] = line[1].toDouble();
        customerY[i] = line[2].toDouble();
        customerDemands[i] = line[3].toInt();
        customerProdType[i] = line[4];
        serviceTime[i] = line[5];
        customerET[i] = convertToMins(line[5].split("-")[0]);
        customerLT[i] = convertToMins(line[5].split("-")[1]);
        lunchTime[i] = line[6];
        if (line[6].toLowerCase()=="none"){
            customera[i] = customerLT[i]+1;
            customerb[i] = customerLT[i]+2;
        } else {
            customera[i] = convertToMins(line[6].split("-")[0]);
            customerb[i] = convertToMins(line[6].split("-")[1]);
        }
        customerPrepTime[i] = line[7].toInt();
        customerPriority[i] = line[8].toInt();
        vehicleOption = line[9].trim();
        if (vehicleOption.toLowerCase()=="none"){
            customerVehicleChoice[i] = all_trucks;
        } else {
            // single choice of Truck
            if (vehicleOption.indexOf(",")==-1) {
                customerVehicleChoice[i] = {truckName2Index[vehicleOption]};
                } 
            // multiple choice of Trucks
            else {
                vehicles = vehicleOption.split(",");
                for [i in 0..vehicles.count()-1] v[i] = truckName2Index[vehicles[i]];
                customerVehicleChoice[i] = v;
                }
        }
        
        i = i + 1;
    }
    nbCustomers = i;

    inFile.close();
}

function skipLines(inFile, nbLines) {
    if (nbLines < 1) return;
    local dump = inFile.readln();
    for[i in 2..nbLines] dump = inFile.readln();
}

function convertToHours(s) {
    hm = s.split(":");
    return hm[0].toDouble() + hm[1].toDouble()/60;
}

function convertToMins(s) {
    hm = s.split(":");
    return hm[0].toDouble()*60 + hm[1].toDouble();
}

function computeDistanceMatrix() {
    n0 = nbCustomers + nbWarehouse + 1;
    // 0 means no visit
    for[i in 0..n0-1]{
        distanceMatrix[0][i] = 0;
        distanceMatrix[i][0] = 0;
    }
    for[i in 1..n0-1]
    {
        distanceMatrix[i][i] = 0;
        if (i <= nbCustomers){
            x_i = customerX[i-1];
            y_i = customerY[i-1];
        } else {
            x_i = warehouseX[i-nbCustomers-1];
            y_i = warehouseY[i-nbCustomers-1];
        }
        for[j in i+1..n0-1] {
            if (j <= nbCustomers){
                x_j = customerX[j-1];
                y_j = customerY[j-1];
            } else {
                x_j = warehouseX[j-nbCustomers-1];
                y_j = warehouseY[j-nbCustomers-1];
            }

            // local localDistance = computeDistance(x_i, y_i, x_j, y_j);
            local localDistance = computeHaversineDistance(x_i, y_i, x_j, y_j);
            // println(x_i, " ", y_i);
            // println(x_j, " ", y_j);
            // println(localDistance);
            distanceMatrix[j][i] = localDistance/1000.0;  // distance in km.
            distanceMatrix[i][j] = localDistance/1000.0;
        }
    }

}

/* 
    https://www.movable-type.co.uk/scripts/latlong.html
    Haversine distance between two points
    cross-verified
*/
function computeHaversineDistance(lat1, lon1, lat2, lon2) {
    PI = 3.141592653589793238;
    R = 6371e3; // metres
    phi1 = lat1 * math.PI/180; // φ, λ in radians
    phi2 = lat2 * math.PI/180;
    dphi = (lat2-lat1) * math.PI/180;
    dlambda = (lon2-lon1) * math.PI/180;

    a = math.sin(dphi/2) * math.sin(dphi/2) +
        math.cos(phi1) * math.cos(phi2) *
        math.sin(dlambda/2) * math.sin(dlambda/2);
    c = 2 * math.atan(math.sqrt(a)/math.sqrt(1-a));

    d = R * c; // in metres
    return d;
}

function computeDistance(x1, x2, y1, y2) {
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}

// check whether y is in the values of map x
// returns 1 if contains
function mapContains(x, y) {
    flag = false;
    for [k in x.keys()] {
        if (x[k] == y){
            flag = true;
        }
    }
    return flag;
}


// check whether y is not in the values of map x
// returns 1 if x does not contain y
function mapNotContains(x, y) {
    flag = true;
    for [k in x.keys()] {
        if (x[k] == y){
            flag = false;
        }
    }
    return flag;
}

