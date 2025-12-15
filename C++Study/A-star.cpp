#include "A-star.h"
#include <iostream>
#include <cstdlib> //abs

int main()
{
	Node n;

	//位置は(2,2)
	n.pos = { 2,2 };

	//スタートからここまで3歩かかったことにする
	n.g = 3.0f;

	//ゴールは(4,5)として、予想距離hを計算
	Vec2i goal{ 4,5 };
	n.h = HeuristicManhattan(n.pos, goal);

	std::cout << "g=" << n.g << "\n";
	std::cout << "h=" << n.h << "\n";
	std::cout << "f=" << n.f() << "\n";   // 3+5=8 のはず

	return 0;
}