#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <random>
#include "graph.h"

// Represents a message in the system, containing the number of hops
// it has taken and its travel time.
struct Message
{
    int hops;
    double travel_time;
};

// Contains statistics for each working thread, including the number of messages received and forwarded,
// the total hops, and the total travel time.
struct ThreadStatistics
{
    int messages_received;
    int messages_forwarded;
    int total_hops;
    double total_travel_time;
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

// Returns a random double value within the specified range.
double rand_range(double min, double max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(min, max);
    return dist(gen);
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
            if (received_message.hops < 20)
            {
                std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(rand_range(100, 500)));

                // Select a random neighbor
                int random_neighbor_index = static_cast<int>(rand_range(0, neighbors.size() - 1));
                auto it = neighbors.begin();
                std::advance(it, random_neighbor_index);
                int target_neighbor = *it;

                all_queues[target_neighbor].send(received_message);
                stats.messages_forwarded++;
            }

            stats.total_hops += received_message.hops;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

void parseArgs(int argc, char **argv, int &s, bool &r, std::string &f)
{
}

// The main function of the program.Initializes the graph, threads,
// message queues, and statistics.Sends initial messages, waits for a
// specified time, then terminates the threadsand prints the statistics.
int main(int argc, char **argv)
{

    // TODO: ADD INPUT

    std::string filename;

    Graph graph("a20.dat");

    // Get the number of nodes from the graph
    const int num_threads = graph.getNodes().size();

    // const int num_threads = 10;
    const int num_messages = 100;

    std::vector<ThreadStatistics> all_stats(num_threads);
    std::vector<MessageQueue> all_queues(num_threads);
    std::vector<std::thread> all_threads;
    std::atomic<bool> terminate(false);

    for (int i = 0; i < num_threads; i++)
    {
        all_threads.emplace_back(workingThread, std::ref(all_stats[i]), std::ref(all_queues[i]), std::ref(terminate), std::cref(graph), graph.getNodes()[i], std::ref(all_queues));
    }

    for (int i = 0; i < num_messages; i++)
    {
        Message initial_message{0, 0};
        all_queues[i % num_threads].send(initial_message);
    }

    std::this_thread::sleep_for(std::chrono::seconds(10));

    terminate = true;
    for (auto &thread : all_threads)
    {
        thread.join();
    }

    for (int i = 0; i < num_threads; i++)
    {
        std::cout << "Thread " << i << " statistics:\n";
        std::cout << "  Messages received: " << all_stats[i].messages_received << '\n';
        std::cout << "  Messages forwarded: " << all_stats[i].messages_forwarded << '\n';
        std::cout << "  Total hops: " << all_stats[i].total_hops << '\n';
        std::cout << "  Total travel time: " << all_stats[i].total_travel_time << '\n';
    }

    return 0;
}
