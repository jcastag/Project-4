#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>

#include <queue>
#include <condition_variable>
#include <random>
#include <cstring>
#include "graph.h"

using std::cout;
using std::endl;
using std::mutex;

// Represents a message in the system, containing the number of hops
// it has taken and its travel time.

struct AntConstants
{
    double startPheremone = 10.0; // starting pheremont
    int pc = 4;                   // power coefficient
    int c = 5;                    // incremental constant
    int hl = 2;                   // dilution half-life
};

struct Message
{
    int hops = 0;
    double travel_time = 0;
    int toId = 0;
    int fromId = 0;
    int lastNodeId = 0;
    std::chrono::time_point<std::chrono::system_clock> timeSent;
    std::chrono::time_point<std::chrono::system_clock> timeReceived;
};

// Contains statistics for each working thread, including the number of messages received and forwarded,
// the total hops, and the total travel time.
struct ThreadStatistics
{
    int messages_received;
    int messages_forwarded;
    int total_hops;
    int messages_kept;
    double total_travel_time;
    std::vector<double> letter_delivery_times;
    std::mutex total_travel_time_mutex;
};

// A thread - safe message queue that allows sending and receiving messages.
class MessageQueue
{
public:
    // Sends a message to the message queue in a thread-safe manner, using a mutex and condition variable.
    void send(const Message &msg)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.push(msg);
        lock.unlock();
        cond_var_.notify_one();
    }

    // Tries to receive a message from the message queue. Returns true if a message is received, false otherwise.
    bool try_receive(Message &msg)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!queue_.empty())
        {
            msg = queue_.front();
            queue_.pop();
            return true;
        }
        return false;
    }

private:
    std::queue<Message> queue_;
    std::mutex mutex_;
    std::condition_variable cond_var_;
};

int findInSet(std::set<int> S, int f)
{
    for (auto a : S)
    {
        if (a == f)
            return f;
    }
    return -1;
}

// Returns a random double value within the specified range.
double rand_range(double min, double max)
{

    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(min, max);
    return dist(gen);
}

int pickAntNode() // add arguments
{

    return 0; // temp return
}

// The main function executed by each thread. Processes messages and forwards them to random neighbors in the graph.

void workingThread(ThreadStatistics &stats, MessageQueue &mq, std::atomic<bool> &terminate, const Graph &graph, int nodeId, std::vector<MessageQueue> &all_queues)
{
    const auto &neighbors = graph.getNeighbors(nodeId);

    while (!terminate)
    {
        Message received_message;
        if (mq.try_receive(received_message))
        {
            // Process the received message
            stats.messages_received++;
            received_message.hops++;
            received_message.travel_time += rand_range(0.01, 0.1);

            // Update the total_travel_time safely
            {
                std::unique_lock<std::mutex> lock(stats.total_travel_time_mutex);
                stats.total_travel_time += received_message.travel_time;
            }

            // Forward message if necessary
            // TODO[] change requirement to stop forwarding be if the recipient id matches the current node
            if (received_message.toId != nodeId)
            {
                std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(rand_range(100, 500)));
                int target_neighbor;
                // Select a random neighbor
                // TODO: is sent to correct node if recipient is neighbor
                if (findInSet(neighbors, received_message.toId) != -1)
                {
                    target_neighbor = received_message.toId;
                }
                else
                {
                    int random_neighbor_index = static_cast<int>(rand_range(0, neighbors.size() - 1));
                    auto it = neighbors.begin();
                    std::advance(it, random_neighbor_index);
                    target_neighbor = *it;
                }
                all_queues[target_neighbor].send(received_message);
                stats.messages_forwarded++;
                stats.total_hops += received_message.hops;
            }
            else
            {
                received_message.timeReceived = std::chrono::system_clock::now();
                double d = std::chrono::duration<double>(std::chrono::duration_cast<std::chrono::milliseconds>(received_message.timeReceived - received_message.timeSent)).count();
                stats.letter_delivery_times.push_back(d);
                stats.messages_kept++;
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(rand_range(1, 500))));
            }
        }
    }
}

void workingAntThread(ThreadStatistics &stats, MessageQueue &mq, std::atomic<bool> &terminate, Graph &graph, int nodeId, std::vector<MessageQueue> &all_queues, AntConstants ac)
{
    const auto &neighbors = graph.getNeighbors(nodeId);
    const double pheremone = ac.startPheremone; // starting pheremone
    const int n = ac.pc;                        // power coefficient
    const int inc = ac.c;                       // incremental constant

    for (auto &neighbor : neighbors)
    {
        graph.setEdgeWeight(nodeId, neighbor, pheremone);
    }

    while (!terminate)
    {
        Message received_message;
        if (mq.try_receive(received_message))
        {
            // Process the received message
            stats.messages_received++;
            received_message.hops++;
            received_message.travel_time += rand_range(0.01, 0.1);
            // Update the total_travel_time safely
            {
                std::unique_lock<std::mutex> lock(stats.total_travel_time_mutex);
                stats.total_travel_time += received_message.travel_time;
            }
            // pheremone incremental increase
            graph.setEdgeWeight(nodeId, received_message.fromId,
                                graph.getEdgeWeight(nodeId, received_message.fromId) + inc);

            // Forward message if necessary
            // TODO[] change requirement to stop forwarding be if the recipient id matches the current node
            if (received_message.toId != nodeId)
            {
                std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(rand_range(100, 500)));
                int target_neighbor;
                // Select a random neighbor
                // TODO: is sent to correct node if recipient is neighbor
                if (findInSet(neighbors, received_message.toId) != -1)
                {
                    target_neighbor = received_message.toId;
                }
                else
                {
                    // DO ANT MATH HERE
                }
                all_queues[target_neighbor].send(received_message);
                stats.messages_forwarded++;
                stats.total_hops += received_message.hops;
            }
            else
            {
                stats.messages_kept++;
                received_message.timeReceived = std::chrono::system_clock::now();
                double d = std::chrono::duration<double>(std::chrono::duration_cast<std::chrono::milliseconds>(received_message.timeReceived - received_message.timeSent)).count();
                stats.letter_delivery_times.push_back(d);
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(rand_range(1, 500))));
            }
        }
    }
}

// thread for making messages for each node
//  TODO[] make messages
//  messages should be sent to random neighbor, goal node should not be a neighbor
//  TODO[] add formula for job spacing
void MessageThread(MessageQueue &mq, std::atomic<bool> &terminate, const Graph &graph, int nodeId, std::vector<MessageQueue> &all_queues)
{
    int lambda = 15;
    int gSize = static_cast<int>(graph.getNodes().size());
    int sent = 0;
    int denominator = (gSize + lambda);
    int i = 0;
    const auto &neighbors = graph.getNeighbors(nodeId);
    while (!terminate)
    {

        int sleep_duration = ((nodeId) * (lambda)) / denominator;

        Message initial_message;
        initial_message.fromId = nodeId;
        initial_message.lastNodeId = nodeId;
        int to = 0;
        // setting to node
        do
        {
            to = static_cast<int>(rand_range(0, graph.getNodes().size()));
        } while (findInSet(neighbors, to) != -1);
        initial_message.toId = to;
        initial_message.timeSent = std::chrono::system_clock::now();
        all_queues[nodeId % graph.getNodes().size()].send(initial_message);
        sent++;
        i++;

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_duration));
        denominator = gSize + (lambda * i);
    }
    int k = 0;
}

bool isValidNumber(char *a)
{
    if (a[0] == '-' || (strlen(a) == 1 && a[0] == '0'))
    {
        std::cout << "seconds parameter cannot be negative or zero." << std::endl;
        return false;
    }
    else
    {
        for (int i = 0; i < strlen(a); i++)
        {
            if (!isdigit(a[i]))
                return false;
        }
        return true;
    }
}

bool isDatFile(std::string f)
{
    std::string end = ".dat";
    std::string::iterator it;
    std::string::iterator it2;
    it = f.end();
    it2 = end.end();
    it2--;
    it--;
    int i = 0;
    do
    {
        if (*it != *it2)
            return false;
        i++;
        it--;
        it2--;
    } while (i < 3);

    return true;
}

// running under windows, cannot use POSIX getopt()
void parseArgs(int argc, char **argv, int &s, bool &r, std::string &f)
{
    if (argc <= 1)
    {
        std::cout << "Not enough arguments!" << std::endl;
    }
    else
    {
        std::string current = "";
        for (int i = 1; i < argc; i++)
        {
            current = argv[i];

            if (current == "-d") // PARSING FOR SECONDS PARAM
            {
                if (isValidNumber(argv[i + 1]))
                {
                    s = atoi(argv[i + 1]);
                    std::cout << s << " seconds." << std::endl;
                }
                else if (!isValidNumber(argv[i + 1]))
                {
                    std::cout << "not valid seconds entry, using default value." << std::endl;
                }
            }
            else if (current == "-r") // PARSING FOR ROUTING PARAM
            {
                std::string a = argv[i + 1];
                if (a == "ant")
                {
                    r = true;
                    std::cout << "ant routing method selected" << std::endl;
                }
                else
                {
                    r = false;
                    if (a == "hot")
                        std::cout << "hot-potato routing method selected" << std::endl;
                    else
                        std::cout << "no proper routing input: selecting hot-potato as default" << std::endl;
                }
            }
            else if (current != "" && isDatFile(current)) // PARSING FOR FILE (REQUIRED PARAM)
            {
                // should be taking in the file name
                f = argv[i];
                std::ifstream file("graph/" + f);
                if (!file.is_open())
                {
                    std::cerr << "Error opening file: " << f << std::endl;
                    // exit(1);
                }
                else
                {
                    cout << "using file: " << f << endl;
                }
            }
        }
        if (r)
        {
            cout << "using ant routing" << endl;
        }
        else
            cout << "using hot-potato routing" << endl;
        cout << endl;
    }
    if (!isDatFile(f))
    {
        std::cout << "No valid file entered! exiting . . ." << std::endl;
        exit(0);
    }
}

void pheremoneEvap(Graph &graph, std::atomic<bool> &terminate, AntConstants ac)
{
    int gSize = graph.getNodes().size();
    while (!terminate)
    {
        // go through each node
        // alter each edge for each node
        for (int i = 0; i < gSize; i++)
        {
            for (auto &e : graph.getEdges(i))
            {
                e.weight *= (1 - ac.hl);
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
        // sleep increment time
    }
}

// The main function of the program.Initializes the graph, threads,
// message queues, and statistics.Sends initial messages, waits for a
// specified time, then terminates the threadsand prints the statistics.
int main(int argc, char **argv)
{

    // argument variables
    int seconds;
    bool isAnt;
    std::string filename;

    AntConstants ant_constants;

    // Parses command line arguments
    parseArgs(argc, argv, seconds, isAnt, filename);

    Graph graph(filename);

    // Get the number of nodes from the graph
    const int num_threads = graph.getNodes().size();

    // TODO change how messages are made
    const int num_messages = 100;

    std::vector<ThreadStatistics> all_stats(num_threads);
    std::vector<MessageQueue> all_queues(num_threads);
    std::vector<std::thread> all_threads;
    std::atomic<bool> terminate(false);

    for (int i = 0; i < num_threads; i++)
    {
        all_threads.emplace_back(MessageThread, std::ref(all_queues[i]), std::ref(terminate), std::cref(graph), graph.getNodes()[i], std::ref(all_queues));
        if (isAnt) // IF ANT ROUTING IS SELECTED
            all_threads.emplace_back(workingAntThread, std::ref(all_stats[i]), std::ref(all_queues[i]), std::ref(terminate), std::ref(graph), graph.getNodes()[i], std::ref(all_queues), ant_constants);

        else
        {
            all_threads.emplace_back(workingThread, std::ref(all_stats[i]), std::ref(all_queues[i]), std::ref(terminate), std::cref(graph), graph.getNodes()[i], std::ref(all_queues));
        }
    }

    if (isAnt)
    { // makes thread for pheremone updates
        // pheremone updates
        int pheremone_update_interval;
        std::thread pheremoneThread(pheremoneEvap, std::ref(graph), std::ref(terminate), ant_constants);
    }

    std::this_thread::sleep_for(std::chrono::seconds(seconds));

    terminate = true;
    int i = 0;
    for (auto &thread : all_threads)
    {
        if (thread.joinable())
            thread.join();
        else
            thread.detach();
        i++;
    }

    for (int i = 0; i < num_threads; i++)
    {
        std::cout << "Thread " << i << " statistics:\n";
        std::cout << "  Messages received: " << all_stats[i].messages_received << '\n';
        std::cout << "  Messages forwarded: " << all_stats[i].messages_forwarded << '\n';
        std::cout << "  Messages kept: " << all_stats[i].messages_kept << '\n';
        std::cout << "  Total hops: " << all_stats[i].total_hops << '\n';
        std::cout << "  Total travel time: " << all_stats[i].total_travel_time << '\n';
        double td = 0;
        for (auto &lt : all_stats[i].letter_delivery_times)
        {
            td += lt;
        }
        std::cout << "  Average letter travel time: " << (td / all_stats[i].letter_delivery_times.size()) << " ms \n";
    }

    return 0;
}
