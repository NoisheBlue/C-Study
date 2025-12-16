#include "A-star.h"
#include <iostream>
#include <cstdlib> //abs

int Astar::AstarAlgo()
{
    GridMap map(10, 7);

    // 壁を作る（例：縦の壁）
    for (int y = 1; y <= 5; ++y)
        map.SetWall(4, y, true);

    // 壁に穴を1つ開ける（ここ通れる）
    map.SetWall(4, 3, false);

    Vec2i start{ 1, 3 };
    Vec2i goal{ 8, 3 };

    Astar astar;
    std::vector<Vec2i> path;

    bool ok = astar.FindPath(map, start, goal, path);

    if (ok)
    {
        std::cout << "Path found! length = " << path.size() << "\n";
        PrintMapWithPath(map, start, goal, path);
    }
    else
    {
        std::cout << "No path.\n";
        PrintMapWithPath(map, start, goal, path); // 経路なしでも表示はできる
    }

    return 0;
}
