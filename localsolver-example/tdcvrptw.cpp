#include "localsolver.h"
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>

using namespace localsolver;
using namespace std;

class Tdcvrptw {
public:
    // LocalSolver
    LocalSolver localsolver;

    // Number of customers
    int nbCustomers;

    // Capacity of the trucks
    int truckCapacity;

    // Latest allowed arrival to depot
    int maxHorizon;

    // Demand for each customer
    vector<int> demandsData;

    // Earliest arrival for each customer
    vector<int> earliestStartData;

    // Latest departure from each customer
    vector<int> latestEndData;

    // Service time for each customer
    vector<int> serviceTimeData;

    // Distance matrix between customers
    vector<vector<double>> distMatrixData;

    // Distance  between customers and depot
    vector<double> distDepotData;

    // Travel time coefficients for each profile
    vector<double> shortDistanceTravelTimeProfile{1.00, 2.50, 1.75, 2.50, 1.00};
    vector<double> mediumDistanceTravelTimeProfile{1.00, 2.00, 1.50, 2.00, 1.00};
    vector<double> longDistanceTravelTimeProfile{1.00, 1.60, 1.10, 1.60, 1.00};
    vector<vector<double>> travelTimeProfileMatrix{shortDistanceTravelTimeProfile, mediumDistanceTravelTimeProfile,
                                                   longDistanceTravelTimeProfile};

    // Distance levels
    vector<int> distanceLevels{10, 25};

    // Intervals of the temporal discretization
    vector<double> timeIntervalSteps{0.0, 0.2, 0.4, 0.6, 0.8, 1.0};

    // Number of time intervals
    int nbTimeIntervals = timeIntervalSteps.size() - 1;

    // Number of distance levels
    int nbDistanceLevels = distanceLevels.size();

    // Travel time between customers for each day part
    vector<vector<vector<double>>> travelTimeData;

    // Time interval index for each time unit
    vector<int> timeToMatrixIdxData;

    // Travel time between customers and depot for each day part
    vector<vector<double>> travelTimeWarehouseData;

    // Number of trucks
    int nbTrucks;

    // Decision variables
    vector<LSExpression> customersSequences;

    // Are the trucks actually used
    vector<LSExpression> trucksUsed;

    // Cumulated lateness in the solution (must be 0 for the solution to be valid)
    LSExpression totalLateness;

    // Number of trucks used in the solution
    LSExpression nbTrucksUsed;

    // Distance traveled by all the trucks
    LSExpression totalDistance;

    Tdcvrptw() {}

    /* Read instance data */
    void readInstance(const string& fileName) { readInputCvrptw(fileName); }

    void solve(int limit) {
        // Declare the optimization model
        LSModel model = localsolver.getModel();

        // Sequence of customers visited by each truck
        customersSequences.resize(nbTrucks);
        for (int k = 0; k < nbTrucks; ++k) {
            customersSequences[k] = model.listVar(nbCustomers);
        }

        // All customers must be visited by exactly one truck
        model.constraint(model.partition(customersSequences.begin(), customersSequences.end()));

        // Create LocalSolver arrays to be able to access them with an "at" operator
        LSExpression demands = model.array(demandsData.begin(), demandsData.end());
        LSExpression earliest = model.array(earliestStartData.begin(), earliestStartData.end());
        LSExpression latest = model.array(latestEndData.begin(), latestEndData.end());
        LSExpression serviceTime = model.array(serviceTimeData.begin(), serviceTimeData.end());
        LSExpression distMatrix = model.array();
        for (int n = 0; n < nbCustomers; ++n) {
            distMatrix.addOperand(model.array(distMatrixData[n].begin(), distMatrixData[n].end()));
        }
        LSExpression travelTime = model.array();
        for (int n = 0; n < nbCustomers; ++n) {
            LSExpression timeMatrix = model.array();
            for (int m = 0; m < nbCustomers; ++m) {
                timeMatrix.addOperand(model.array(travelTimeData[n][m].begin(), travelTimeData[n][m].end()));
            }
            travelTime.addOperand(timeMatrix);
        }
        LSExpression timeToMatrixIdx = model.array(timeToMatrixIdxData.begin(), timeToMatrixIdxData.end());
        LSExpression distDepot = model.array(distDepotData.begin(), distDepotData.end());
        LSExpression travelTimeWarehouse = model.array();
        for (int n = 0; n < nbCustomers; ++n) {
            travelTimeWarehouse.addOperand(
                model.array(travelTimeWarehouseData[n].begin(), travelTimeWarehouseData[n].end()));
        }

        trucksUsed.resize(nbTrucks);
        vector<LSExpression> distRoutes(nbTrucks), endTime(nbTrucks), homeLateness(nbTrucks), lateness(nbTrucks);

        for (int k = 0; k < nbTrucks; ++k) {
            LSExpression sequence = customersSequences[k];
            LSExpression c = model.count(sequence);

            // A truck is used if it visits at least one customer
            trucksUsed[k] = c > 0;

            // The quantity needed in each route must not exceed the truck capacity
            LSExpression demandLambda =
                model.createLambdaFunction([&](LSExpression j) { return demands[j]; });
            LSExpression routeQuantity = model.sum(sequence, demandLambda);
            model.constraint(routeQuantity <= truckCapacity);

            // Distance traveled by truck k
            LSExpression distLambda = model.createLambdaFunction(
                [&](LSExpression i) { return model.at(distMatrix, sequence[i - 1], sequence[i]); });
            distRoutes[k] = model.sum(model.range(1, c), distLambda) +
                            model.iif(c > 0, distDepot[sequence[0]] + distDepot[sequence[c - 1]], 0);

            // End of each visit according to the traffic
            LSExpression endTimeLambda = model.createLambdaFunction([&](LSExpression i, LSExpression prev) {
                return model.max(earliest[sequence[i]],
                                 model.iif(i == 0, model.at(travelTimeWarehouse, sequence[0], timeToMatrixIdx[0]),
                                           prev + model.at(travelTime, sequence[i - 1], sequence[i],
                                                           timeToMatrixIdx[model.round(prev)]))) +
                       serviceTime[sequence[i]];
            });

            endTime[k] = model.array(model.range(0, c), endTimeLambda);

            // Arriving home after max horizon
            homeLateness[k] = model.iif(trucksUsed[k],
                                        model.max(0, endTime[k][c - 1] +
                                                         model.at(travelTimeWarehouse, sequence[c - 1],
                                                                  timeToMatrixIdx[model.round(endTime[k][c - 1])]) -
                                                         maxHorizon),
                                        0);

            // Completing visit after latest end
            LSExpression lateLambda = model.createLambdaFunction(
                [&](LSExpression i) { return model.max(0, endTime[k][i] - latest[sequence[i]]); });
            lateness[k] = homeLateness[k] + model.sum(model.range(0, c), lateLambda);
        }

        // Total lateness
        totalLateness = model.sum(lateness.begin(), lateness.end());

        // Total number of trucks used
        nbTrucksUsed = model.sum(trucksUsed.begin(), trucksUsed.end());

        // Total distance traveled (convention in Solomon's instances is to round to 2 decimals)
        totalDistance = model.round(100 * model.sum(distRoutes.begin(), distRoutes.end())) / 100;

        // Objective: minimize the lateness, then the number of trucks used, then the distance traveled
        model.minimize(totalLateness);
        model.minimize(nbTrucksUsed);
        model.minimize(totalDistance);

        model.close();

        // Parameterize the solver
        localsolver.getParam().setTimeLimit(limit);

        localsolver.solve();
    }

    /* Write the solution in a file with the following format:
     *  - number of trucks used and total distance
     *  - for each truck the customers visited (omitting the start/end at the depot) */
    void writeSolution(const string& fileName) {
        ofstream outfile;
        outfile.exceptions(ofstream::failbit | ofstream::badbit);
        outfile.open(fileName.c_str());

        outfile << nbTrucksUsed.getValue() << " " << totalDistance.getDoubleValue() << endl;
        for (int k = 0; k < nbTrucks; ++k) {
            if (trucksUsed[k].getValue() != 1)
                continue;
            // Values in sequence are in 0...nbCustomers. +1 is to put it back in 1...nbCustomers+1
            // as in the data files (0 being the depot)
            LSCollection customersCollection = customersSequences[k].getCollectionValue();
            for (int i = 0; i < customersCollection.count(); ++i) {
                outfile << customersCollection[i] + 1 << " ";
            }
            outfile << endl;
        }
    }

private:
    // The input files follow the "Solomon" format
    void readInputCvrptw(const string& fileName) {
        ifstream infile(fileName.c_str());
        if (!infile.is_open()) {
            throw std::runtime_error("File cannot be opened.");
        }

        string str;
        long tmp;

        int depotX, depotY;
        vector<int> customersX;
        vector<int> customersY;

        getline(infile, str);
        getline(infile, str);
        getline(infile, str);
        getline(infile, str);

        infile >> nbTrucks;
        infile >> truckCapacity;

        getline(infile, str);
        getline(infile, str);
        getline(infile, str);
        getline(infile, str);

        infile >> tmp;
        infile >> depotX;
        infile >> depotY;
        infile >> tmp;
        infile >> tmp;
        infile >> maxHorizon;
        infile >> tmp;

        while (infile >> tmp) {
            int cx, cy, demand, ready, due, service;
            infile >> cx;
            infile >> cy;
            infile >> demand;
            infile >> ready;
            infile >> due;
            infile >> service;

            customersX.push_back(cx);
            customersY.push_back(cy);
            demandsData.push_back(demand);
            earliestStartData.push_back(ready);
            latestEndData.push_back(due + service); // in input files due date is meant as latest start time
            serviceTimeData.push_back(service);
        }

        nbCustomers = customersX.size();

        computeDistanceMatrix(depotX, depotY, customersX, customersY);

        infile.close();
    }

    // Compute the distance matrix
    void computeDistanceMatrix(int depotX, int depotY, const vector<int>& customersX, const vector<int>& customersY) {
        distMatrixData.resize(nbCustomers);
        travelTimeData.resize(nbCustomers);
        timeToMatrixIdxData.resize(maxHorizon);
        travelTimeWarehouseData.resize(nbCustomers);
        for (int i = 0; i < nbCustomers; ++i) {
            distMatrixData[i].resize(nbCustomers);
            travelTimeData[i].resize(nbCustomers);
            for (int j = 0; j < nbCustomers; ++j) {
                travelTimeData[i][j].resize(nbTimeIntervals);
            }
        }
        for (int i = 0; i < nbCustomers; ++i) {
            travelTimeWarehouseData[i].resize(nbTimeIntervals);
        }
        for (int i = 0; i < nbCustomers; ++i) {
            distMatrixData[i][i] = 0;
            for (int k = 0; k < nbTimeIntervals; ++k) {
                travelTimeData[i][i][k] = 0;
            }
            for (int j = i + 1; j < nbCustomers; ++j) {
                double distance = computeDist(customersX[i], customersX[j], customersY[i], customersY[j]);
                distMatrixData[i][j] = distance;
                distMatrixData[j][i] = distance;

                int profileIdx = getProfile(distance);
                for (int k = 0; k < nbTimeIntervals; ++k) {
                    double localTravelTime = travelTimeProfileMatrix[profileIdx][k] * distance;
                    travelTimeData[i][j][k] = localTravelTime;
                    travelTimeData[j][i][k] = localTravelTime;
                }
            }
        }

        for (int i = 0; i < nbTimeIntervals; ++i) {
            int timeStepStart = (int)round(timeIntervalSteps[i] * maxHorizon);
            int timeStepEnd = (int)round(timeIntervalSteps[i + 1] * maxHorizon);
            for (int j = timeStepStart; j < timeStepEnd; ++j) {
                timeToMatrixIdxData[j] = i;
            }
        }

        distDepotData.resize(nbCustomers);
        for (int i = 0; i < nbCustomers; ++i) {
            double distance = computeDist(depotX, customersX[i], depotY, customersY[i]);
            distDepotData[i] = distance;

            int profileIdx = getProfile(distance);
            for (int j = 0; j < nbTimeIntervals; ++j) {
                double localTravelTimeWarehouse = travelTimeProfileMatrix[profileIdx][j] * distance;
                travelTimeWarehouseData[i][j] = localTravelTimeWarehouse;
            }
        }
    }

    double computeDist(int xi, int xj, int yi, int yj) {
        return sqrt(pow((double)xi - xj, 2) + pow((double)yi - yj, 2));
    }

    int getProfile(double distance) {
        int idx = 0;
        while (idx < nbDistanceLevels && distance > distanceLevels[idx]) {
            idx += 1;
        }
        return idx;
    }
};

int main(int argc, char** argv) {
    if (argc < 2) {
        cerr << "Usage: tdcvrptw inputFile [outputFile] [timeLimit] [nbTrucks]" << endl;
        return 1;
    }

    const char* instanceFile = argv[1];
    const char* solFile = argc > 2 ? argv[2] : NULL;
    const char* strTimeLimit = argc > 3 ? argv[3] : "20";

    try {
        Tdcvrptw model;
        model.readInstance(instanceFile);
        model.solve(atoi(strTimeLimit));
        if (solFile != NULL)
            model.writeSolution(solFile);
        return 0;
    } catch (const exception& e) {
        cerr << "An error occurred: " << e.what() << endl;
        return 1;
    }
}