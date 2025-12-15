#pragma once
#include <vector>
#include <cstdint>
#include <cassert>
#include <queue>
#include <algorithm> // reverse
#include <limits> //infinityを使うため


struct Vec2i
{
	int x = 0;
	int y = 0;

	//座標の一致判定
	bool operator==(const Vec2i& other) const {
		return x == other.x && y == other.y;
	}
};

//１マス分の探索情報をまとめる箱

struct Node {

	//このノードが表すマスの位置

	Vec2i pos;

	// g; スタートからこのますまでのコスト(歩いた歩数)
	//まだ未訪問のマスは無限大にしておくとべんり
	float g = std::numeric_limits<float>::infinity();

	//h: このマスからゴールまでの推定コスト(だいたいの距離)
	float h = 0.0f;

	// parent: どのマスから来たか
	//パスを復元するときに使う
	//（-1,-1）は親がまだない目印
	Vec2i parent{ -1,-1 };

	//f: A*が優先順に使う値
	//f = g + h(総合評価)
	float f() const {
		//constはこの関数はNodeの中身を変えませんの約束
		return g + h;
	}


};

//(a)から(b)までの大体の距離を返す
//上下左右移動なら[dx]+[dy]が定番
inline float HeuristicManhattan(const Vec2i& a, const Vec2i& b)
{
	int dx = std::abs(a.x - b.x); //xの差の絶対値(符号をなくした数字だけ)
	int dy = std::abs(a.y - b.y); //yの差の絶対値
	return static_cast<float>(dx + dy);
}

inline int ToIndex(int x, int y, int width)
{
	return y * width + x;
}

class GridMap
{
public:
	//w.hのサイズでマップを作る
	//初期状態は全部0(=通れるように)しておく
	GridMap(int w, int h) :w_(w), h_(h), cells_(w* h, 0) {
		//変なサイズがきたら開発中に気付けるように
		assert(w_ > 0 && h_ > 0);
	}

	int Width() const { return w_; }
	int Height() const { return h_; }

	//(x,y)がマップの範囲内か？
	bool InBounds(int x, int y) const {
		//0<= x < w_ かつ 0 <= y < h
		return (0 <= x && x < w_) && (0 <= y && y < h_);

	}

	// 壁を置く/外す
	// wall=true なら壁、falseなら通れる
	void SetWall(int x, int y, bool wall)
	{
		// 範囲外アクセスはバグの温床なのでここで止める
		assert(InBounds(x, y));

		// 壁なら 1、通路なら 0
		cells_[Index(x, y)] = wall ? 1 : 0;
	}

	// 壁かどうか（範囲外は壁扱いにすると探索が楽）
	bool IsWall(int x, int y) const
	{
		// 範囲外は「通れない」にする（安全）
		if (!InBounds(x, y)) return true;

		return cells_[Index(x, y)] == 1;
	}

	//壁かどうか(範囲外は壁扱いにすると探索が楽)
	bool Passable(int x, int y) const {
		//範囲内で、壁じゃなければ通れる
		return InBounds(x, y) && !IsWall(x, y);
	}

private:

	int w_ = 0;
	int h_ = 0;

	//マップデータ本体
	//1マスを1byteで保存
	std::vector<uint8_t> cells_;

	//(x,y)を一次元配列の添字に変換する
	//y行目の戦闘はy*w_、そにこxを足す

	int Index(int x, int y) const
	{
		return y * w_ + x;

	}
};

class Astar
{
public:
	//==============================
	// 経路探索のメイン関数
	//==============================
	// map  : 壁があるマップ
	// start: スタート座標
	// goal : ゴール座標
	// outPath: 見つかった経路（start→goal の順で座標が入る）
	//
	// return:
	//   true  = 経路が見つかった
	//   false = 見つからなかった（壁で塞がれてる等）
	bool FindPath(const GridMap& map, const Vec2i& start, const Vec2i& goal,
		std::vector<Vec2i>& outPath)
	{
		//前回の結果が残らないように最初に空にする
		outPath.clear();

		//スタートとゴールは「通れるマス」じゃないと探索できない
		if (!map.Passable(start.x, start.y)) return false;
		if (!map.Passable(goal.x, goal.y)) return false;

		//マップサイズ取得
		const int w = map.Width();
		const int h = map.Height();

		//==============================
		// 1) ノード配列を作る（全マス分）
		//==============================
		// nodes[idx] が「そのマスの記録帳」
		// 記録するもの：g/h/親 など
		std::vector<Node> nodes(w * h);

		//各マスの座標pos だけ先に埋めておく(見やすさと安全のため)
		for (int y = 0; y < h; y++) {
			for (int x = 0; x < w; x++) {
				nodes[ToIndex(x, y, w)].pos = { x,y };
			}
		}

		//==============================
	   // 2) Closed（確定済み）フラグ
	   //==============================
	   // closed[idx] = true のマスは「最短確定した」ので二度と見ない
		std::vector<bool> closed(w * h, false);

		//==============================
	   // 3) Open（候補）を管理する優先度付きキュー
	   //==============================
	   // Openには「次に調べるべき有望なマス」を入れる
	   // 取り出すときは f が最小のものを取り出したい
	   //
	   // priority_queue は「大きいものが先に出る」ので
	   // 比較を反転して「小さい f が先に出る」ようにする

		struct OpenItem
		{
			int idx; //nodesの添え字(このマスのID)
			float f; //優先順位に使うf値(g+h)
		};

		struct Compare
		{
			bool operator()(const OpenItem& a, const OpenItem& b) const
			{
				//fが小さいものを先に出したいので
				//aのほうが後ろになる条件をreturnする
				return a.f > b.f;

			}
		};

		std::priority_queue<OpenItem, std::vector<OpenItem>, Compare> open;

		//==============================
		// 4) スタートノードを初期化してOpenへ
		//==============================

		const int startIdx = ToIndex(start.x, start.y, w);
		const int goalIdx = ToIndex(goal.x, goal.y, w);

		//スタートからスタートへのコストは0
		nodes[startIdx].h = 0.0f;

		// スタート→ゴールの「予想距離」
		nodes[start].h = HeuristicManhattan(start, goal);

		// スタートは親が存在しない
		nodes[startIdx].parent = { -1, -1 };

		// Openに入れる（最初の探索候補）
		open.push({ startIdx, nodes[startIdx].f() });

		//==============================
		// 5) 移動方向（今回は上下左右の4方向）
		//==============================
		const Vec2i dirs[4] = { {1,0}, {-1,0}, {0,1}, {0,-1} };


	}
};

