import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Scanner;

/**
 * Calculates the shortest cycle which visits every node of a graph.
 *
 * @author Andrei Muntean
 */
public class TravelingSalesman
{
    // Represents the number of nodes in the graph.
    private static int nodeCount;

    // Represents the number of edges in the graph.
    private static int edgeCount;

    // A value of the form 2^n - 1. Its binary representation is a string of ones. 
    private static int allNodes;

    // The value neighbors[node] stores the neighbors of node.
    private static ArrayList<Integer>[] neighbors;

    // The value costs[firstNode][secondNode] stores the cost of getting from firstNode to secondNode.
    private static int[][] costs;

    // The value dp[node][visitedNodes] stores the minimum cost of getting to node and having visited visitedNodes.
    private static int[][] dp;

    // The value paths[lastNode][visitedNodes] points to secondLastNode from paths[secondLastNode][visitedNodesMinusLastNode].
    private static int[][] paths;

    // Calculates the unsigned minimum of two numbers.
    // Negative numbers will be read as Integer.MAX_VALUE.
    private static int unsignedMin(int firstValue, int secondValue)
    {
        firstValue = firstValue < 0 ? Integer.MAX_VALUE : firstValue;
        secondValue = secondValue < 0 ? Integer.MAX_VALUE : secondValue;

        return Math.min(firstValue, secondValue);
    }

    // Gets the minimum cost of getting to node and having visited visitedNodes.
    private static int getMinimumCost(int node, int visitedNodes)
    {
        return dp[node][visitedNodes];
    }

    // Updates the minimum cost of getting to node and having visited visitedNodes.
    private static void updateMinimumCost(int node, int visitedNodes, int cost)
    {
        dp[node][visitedNodes] = unsignedMin(dp[node][visitedNodes], cost);
    }

    // Adds a path which ends in newNode. Creates it based on a previous path visitedNodes which ends in node.
    private static void addPath(int newNode, int node, int visitedNodes)
    {
        paths[newNode][visitedNodes + (1 << newNode)] = node;
    }

    // Gets the second last node of the path visitedNodes which ends in lastNode.
    private static int getSecondLastNodeOfPath(int lastNode, int visitedNodes)
    {
        return paths[lastNode][visitedNodes];
    }

    // Gets a list with the neighbors of node.
    private static ArrayList<Integer> getNeighbors(int node)
    {
        return neighbors[node];
    }

    // Adds an edge.
    private static void addEdge(int firstNode, int secondNode, int cost)
    {
        // Sets the neighbors.
        neighbors[firstNode].add(secondNode);
        neighbors[secondNode].add(firstNode);

        // Sets the cost.
        costs[firstNode][secondNode] = cost;
        costs[secondNode][firstNode] = cost;
    }

    // Initializes all variables.
    private static void initialize()
    {
        allNodes = (1 << nodeCount) - 1;
        
        // Initializes the cost matrix.
        costs = new int[nodeCount][nodeCount];

        for (int firstIndex = 0; firstIndex < nodeCount; ++firstIndex)
        {
            for (int secondIndex = 0; secondIndex < nodeCount; ++secondIndex)
            {
                costs[firstIndex][secondIndex] = Integer.MAX_VALUE;
            }
        }

        // Initializes the dp matrix.
        dp = new int[nodeCount][allNodes + 1];

        for (int nodeIndex = 0; nodeIndex < nodeCount; ++nodeIndex)
        {
            for (int visitedNodes = 0; visitedNodes <= allNodes; ++visitedNodes)
            {
                dp[nodeIndex][visitedNodes] = Integer.MAX_VALUE;
            }
        }

        // Initializes the array of neighbors.
        neighbors = new ArrayList[nodeCount];

        for (int nodeIndex = 0; nodeIndex < nodeCount; ++nodeIndex)
        {
            neighbors[nodeIndex] = new ArrayList<Integer>();
        }

        // Initializes the paths.
        paths = new int[nodeCount][allNodes + 1];
    }

    // Calculates the values of the matrix dp, namely, the minimum cost of getting to a node and having visited the specified nodes.
    private static void calculateCosts()
    {
        // Starts from the first node (so index 0). The origin is irrelevant because it is a cycle.
        // The minimum cost to have traveled from the first node to the first node is 0.
        updateMinimumCost(0, 1, 0);

        // Tries every combination of nodes.
        // Both values are nodeCount-bit binary numbers. Each 1 represents a visited node.
        // The least significant bit is the first node and the most significant bit is the last node.
        // Starts from 000...001 to 111...111.
        for (int visitedNodes = 1; visitedNodes <= allNodes; ++visitedNodes)
        {
            for (int node = 0; node < nodeCount; ++node)
            {
                // Determines whether we can get to node via visitedNodes.
                if (getMinimumCost(node, visitedNodes) == Integer.MAX_VALUE)
                {
                    // Skips it if it is not possible.
                    continue;
                }

                // Attempts to visit neighboring nodes.
                for (int neighbor : getNeighbors(node))
                {
                    // Assuming we have visited the first, second, fourth and sixth node, we'd have:
                    // visitedNodes == ...00101011.
                    // If the current neighbor is the fifth one (so index 4), then:
                    // visitedNodesPlusThisNeighbor == ...00111011.
                    int visitedNodesPlusThisNeighbor = visitedNodes | 1 << neighbor;

                    // Determines whether this neighbor has been visited before.
                    if (visitedNodes == visitedNodesPlusThisNeighbor)
                    {
                        // Skips it if it has been visited before.
                        continue;
                    }

                    // Updates the minimum cost of getting to this neighbor.
                    // If the currently known minimum cost is smaller, the minimum cost will not change.
                    updateMinimumCost(neighbor, visitedNodesPlusThisNeighbor, getMinimumCost(node, visitedNodes) + costs[node][neighbor]);

                    // Stores the path.
                    addPath(neighbor, node, visitedNodes);
                }
            }
        }
    }

    // Gets the minimum cost of traversing the entire graph and returning to the origin.
    private static int getMinimumCost()
    {
        int minimumCost = Integer.MAX_VALUE;

        for (int node = 1; node < nodeCount; ++node)
        {
            minimumCost = unsignedMin(minimumCost, getMinimumCost(node, allNodes) + costs[node][0]);
        }

        return minimumCost;
    }

    // Gets the shortest cycle which traverses the entire graph.
    private static ArrayList<int[]> getShortestCycle()
    {
        ArrayList<int[]> shortestCycle = new ArrayList<int[]>(nodeCount);
        int minimumCost = getMinimumCost();

        for (int node = 1; node < nodeCount; ++node)
        {
            // Determines whether this is the end node of a path with a minimal cost.
            if (dp[node][allNodes] + costs[node][0] == minimumCost)
            {
                int visitedNodes = allNodes;

                while (node != 0)
                {
                    int visitedNodesMinusThisNode = visitedNodes - (1 << node);

                    // Gets the next node.
                    shortestCycle.add(new int[] { node, node = getSecondLastNodeOfPath(node, visitedNodes) });

                    // Removes this node from the binary number of visited nodes.
                    visitedNodes = visitedNodesMinusThisNode;
                }

                // Completes the cycle.
                shortestCycle.add(new int[] { node, shortestCycle.get(0)[0] });

                // Stops searching for nodes.
                break;
            }
        }

        return shortestCycle;
    }

    // Reads input from the specified path. Reads from standard input if the specified path is the empty string.
    private static void read(String path) throws IOException
    {
        try (Scanner scanner = path.isEmpty() ? new Scanner(System.in) : new Scanner(new FileReader(path)))
        {
            nodeCount = scanner.nextInt();
            edgeCount = scanner.nextInt();
            initialize();

            // Reads the edges.
            for (int edgeIndex = 0; edgeIndex < edgeCount; ++edgeIndex)
            {
                int firstNode = scanner.nextInt();
                int secondNode = scanner.nextInt();
                int cost = scanner.nextInt();

                addEdge(firstNode, secondNode, cost);
            }
        }
    }

    // Writes the result to the specified path. Outputs to standard output if the specified path is the empty string.
    private static void write(String path) throws IOException
    {
        try (PrintWriter printWriter = path.isEmpty() ? new PrintWriter(System.out) : new PrintWriter(new FileWriter(path)))
        {
            int minimumCost = getMinimumCost();

            if (minimumCost < Integer.MAX_VALUE)
            {
                printWriter.println(minimumCost);
            
                for (int[] pair : getShortestCycle())
                {
                    printWriter.printf("(%d, %d) ", pair[0], pair[1]);
                }

                printWriter.println();
            }
            else
            {
                printWriter.println("Infinity");
            }
        }
    }

    public static void main(String[] args) throws IOException
    {
        read(args != null && args.length > 0 ? args[0] : "");
        calculateCosts();
        write(args != null && args.length > 1 ? args[1] : "");
    }
}