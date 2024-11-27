#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <queue>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "cg_interfaces/srv/move_cmd.hpp"
#include "cg_interfaces/srv/get_map.hpp" // Inclui a interface para o serviço de mapa

using namespace std;

struct Node
{
    int x, y;
    float cost, heuristic;
    Node *parent;

    Node(int x, int y, float cost = 0, float heuristic = 0, Node *parent = nullptr)
        : x(x), y(y), cost(cost), heuristic(heuristic), parent(parent) {}

    float totalCost() const
    {
        return cost + heuristic;
    }

    bool operator>(const Node &other) const
    {
        return totalCost() > other.totalCost();
    }
};

// Função para verificar se o movimento é válido
bool isValidMove(int x, int y, const vector<vector<char>> &map)
{
    return x >= 0 && x < map.size() && y >= 0 && y < map[0].size() && map[x][y] != 'b';
}

// Implementação do algoritmo do A*
vector<Node> aStarSearch(const vector<vector<char>> &map, pair<int, int> start, pair<int, int> goal)
{
    priority_queue<Node, vector<Node>, greater<Node>> openList;
    vector<vector<bool>> visited(map.size(), vector<bool>(map[0].size(), false));

    openList.emplace(start.first, start.second, 0, abs(goal.first - start.first) + abs(goal.second - start.second));

    while (!openList.empty())
    {
        Node current = openList.top();
        openList.pop();

        if (visited[current.x][current.y])
            continue;
        visited[current.x][current.y] = true;

        if (current.x == goal.first && current.y == goal.second)
        {
            vector<Node> path;
            while (current.parent)
            {
                path.push_back(current);
                current = *current.parent;
            }
            reverse(path.begin(), path.end());
            return path;
        }

        for (auto &dir : vector<pair<int, int>>{{0, 1}, {1, 0}, {0, -1}, {-1, 0}})
        {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;

            if (isValidMove(nx, ny, map) && !visited[nx][ny])
            {
                float newCost = current.cost + 1;
                float heuristic = abs(goal.first - nx) + abs(goal.second - ny);
                openList.emplace(nx, ny, newCost, heuristic, new Node(current));
            }
        }
    }

    return {}; // Retorna um caminho vazio se não houver solução
}

// Função para movimentação do bloco azul
string getMoveDirection(pair<int, int> current, pair<int, int> next)
{
    int dx = next.first - current.first;
    int dy = next.second - current.second;

    if (dx == 0 && dy == 1)
        return "right";
    if (dx == 1 && dy == 0)
        return "down";
    if (dx == 0 && dy == -1)
        return "left";
    if (dx == -1 && dy == 0)
        return "up";

    return "";
}

// Cria os clinetes do ROS2
class MoveClient : public rclcpp::Node
{
public:
    MoveClient() : Node("move_client")
    {
        move_client_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        map_client_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");

        while (!move_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Esperando pelo serviço 'move_command'...");
        }

        while (!map_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Esperando pelo serviço 'get_map'...");
        }
    }
    // Envia os comandos para movimentação
    bool sendMoveCommand(const string &direction)
    {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direction;

        auto result_future = move_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Comando '%s' enviado com sucesso.", direction.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Falha ao enviar comando '%s'.", direction.c_str());
            return false;
        }
    }

    vector<vector<char>> getMapFromService()
    {
        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        auto result_future = map_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();

            // Reconstruir o mapa a partir dos dados achatados
            vector<vector<char>> map;
            if (result->occupancy_grid_shape.size() == 2)
            {
                uint8_t rows = result->occupancy_grid_shape[0];
                uint8_t cols = result->occupancy_grid_shape[1];

                if (rows * cols == result->occupancy_grid_flattened.size())
                {
                    for (uint8_t i = 0; i < rows; ++i)
                    {
                        vector<char> row;
                        for (uint8_t j = 0; j < cols; ++j)
                        {
                            row.push_back(result->occupancy_grid_flattened[i * cols + j][0]);
                        }
                        map.push_back(row);
                    }
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Inconsistência entre tamanho do mapa e dimensões fornecidas.");
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Formato inválido das dimensões do mapa.");
            }

            return map;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Falha ao obter mapa do serviço.");
            return {};
        }
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr move_client_;
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr map_client_;
};


// Começa função que executa as funções de movimentações e atulizações das posições.
void moveWithRecalculation(MoveClient &node, vector<vector<char>> &map, pair<int, int> &current, pair<int, int> goal)
{
    vector<Node> path;
    while (true)
    {
        path = aStarSearch(map, current, goal);

        if (path.empty())
        {
            RCLCPP_ERROR(node.get_logger(), "Não há caminho para o objetivo.");
            break;
        }

        pair<int, int> next = {path[0].x, path[0].y};
        string direction = getMoveDirection(current, next);

        if (!direction.empty())
        {
            if (node.sendMoveCommand(direction))
            {
                current = next;
            }
        }

        // Atualize o mapa a cada movimento
        map = node.getMapFromService();

        if (map.empty())
        {
            RCLCPP_ERROR(node.get_logger(), "Falha ao obter o mapa atualizado.");
            break;
        }

        if (current == goal)
        {
            RCLCPP_INFO(node.get_logger(), "Objetivo alcançado!");
            break;
        }

        rclcpp::sleep_for(chrono::milliseconds(500));
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = make_shared<MoveClient>();

    vector<vector<char>> map = node->getMapFromService();
    if (map.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "Mapa inicial não pôde ser carregado.");
        return 1;
    }

        pair<int, int> start = {1, 1};
        pair<int, int> goal = {18, 18}; // Ajuste a posição do objetivo

        pair<int, int> current = start;
        moveWithRecalculation(*node, map, current, goal);

    rclcpp::shutdown();
    return 0;
}
