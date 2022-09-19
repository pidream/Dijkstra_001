#include <stdio.h>

#define INF 10000

#define TRUE 1
#define FALSE 0

#define NOWALL	0	//0 壁がない場合の値
#define WALL	1	//1 壁がある場合の値

#define STRAIGHT 90
#define DIAGONAL 64

typedef struct
{
	unsigned char north;	//北の壁情報
	unsigned char east;	//東の壁情報
	unsigned char south;	//南の壁情報
	unsigned char west;	//西の壁情報
}twall;			//壁情報を格納する構造体

typedef struct
{
	unsigned char x;	//
	unsigned char y;	//
	unsigned char dir;	//
}tindex;			//

tindex index[4][4][2]; 
twall wall[4][4];	//壁の情報を格納する構造体配列

int dijkstra(int ,int,int);
void setwall(void);

int main(void){
	int goalx,goaly,goaldir;
	int nodex,nodey,nodedir;
	int tmpx,tmpy,tmpdir;


	//迷路情報の読み込み
	setwall();

	//ゴール区間(3,3)に接するnode(区画3,2の東側)を設定
	goalx=2;
	goaly=3;
	goaldir=1;

	/* ダイクストラ法で最短距離を求める */
	printf("start(0,0:0)からgoal(%d,%d:%d)までの距離: %d mm\n",goalx,goaly,goaldir, dijkstra(goalx,goaly,goaldir) );


	/* 経路を表示(goal区画に接するノードからスタート地点のノードまで) */
	printf("goal(%d,%d:%d)からstart(0,0:0)までの経路\n",goalx,goaly,goaldir);
	nodex = goalx;
	nodey = goaly;
	nodedir = goaldir;
	printf("%d,%d:%d", nodex,nodey,nodedir);
	while(1){
		tmpx=nodex;
		tmpy=nodey;
		tmpdir=nodedir;
		nodex = index[tmpx][tmpy][tmpdir].x;
		nodey = index[tmpx][tmpy][tmpdir].y;
		nodedir = index[tmpx][tmpy][tmpdir].dir;
		printf(" -> %d,%d:%d", nodex,nodey,nodedir);
		if (nodex == 0 && nodey==0 && nodedir==0) break;
	}

	printf("\n\n");
	return 0;
}
 
 
 /* *************************************  ************************************* */
int dijkstra(int goalx,int goaly,int goaldir){
	unsigned short min; //
	unsigned char vx,vy,vdir; //
	unsigned short COST[4][4][2];
	unsigned char USED[4][4][2];//距離確定フラグ用


	/* 初期化 */
	for(int cx = 0; cx < 4 ; cx++){
		for(int cy = 0; cy < 4 ; cy++){
			COST[cx][cy][0] = INF; //スタートからの距離を無限大で初期化
			USED[cx][cy][0] = FALSE;//未確定ノードのとして初期化
			COST[cx][cy][1] = INF; 
			USED[cx][cy][1] = FALSE; 
		}
	}
	COST[0][0][0] = 0; // スタート地点は座標0,0 の北のノード


	while(1){

		/* 未確定ノード(!USED[cx][cy][dir])の中から距離が最も小さいノードを選ぶ */
		min = INF;
		for(int cx = 0; cx < 4 ; cx++){
			for(int cy = 0; cy < 4 ; cy++){
	 			for(int dir=0 ;dir<2;dir++){
					if(USED[cx][cy][dir]==FALSE && min > COST[cx][cy][dir] ) {
						min = COST[cx][cy][dir];
						vx = cx;
						vy = cy;
						vdir = dir;
					}
	 			}
		 	}
		}
		USED[vx][vy][vdir] = TRUE; //このノードの距離は確定
		printf("最小ﾉ-ﾄﾞ:%d,%d %d 値:%d \n",vx,vy,vdir,min);

		if(min == INF){printf("goalまでの経路無し\n");return INF;}

		/* 最小ノードがゴールノードになった（これ以上の計算は不要） */
		if(vx == goalx   && vy == goaly   && vdir == goaldir){return COST[vx][vy][vdir];}//ゴールへの入口（ゴール区間に接するノード）


		/* 今確定したノードに直接つながっているノードに関して、今確定したノードを経由して到達する距離を計算して、より短い距離になれば更新する */
		if(vdir==0){//北or南向き
			if(wall[vx][vy+1].north == NOWALL && COST[vx  ][vy+1][0] > COST[vx][vy][0] + STRAIGHT ){//北に進める
			COST[vx][vy+1][0] = COST[vx][vy][0] + STRAIGHT;
				printf("北に行ける %d \n",COST[vx][vy+1][0]);
				//最短ルート復元用に前のノードへのポインタをindexに格納する
				index[vx][vy+1][0].x = vx;
				index[vx][vy+1][0].y = vy;
				index[vx][vy+1][0].dir = 0;
			}
			if(wall[vx][vy+1].east  == NOWALL && COST[vx  ][vy+1][1] > COST[vx][vy][0] + DIAGONAL ){//北東に進める
				COST[vx][vy+1][1] = COST[vx][vy][0] + DIAGONAL;
				printf("北東に行ける %d \n",COST[vx][vy+1][1]);
				index[vx][vy+1][1].x = vx;
				index[vx][vy+1][1].y = vy;
				index[vx][vy+1][1].dir = 0;
			}
			if(wall[vx][vy+1].west  == NOWALL && COST[vx-1][vy+1][1] > COST[vx][vy][0] + DIAGONAL ){//北西に進める
				COST[vx-1][vy+1][1] = COST[vx][vy][0] + DIAGONAL;
				printf("北西に行ける %d \n",COST[vx-1][vy+1][1]);
				index[vx-1][vy+1][1].x = vx;
				index[vx-1][vy+1][1].y = vy;
				index[vx-1][vy+1][1].dir = 0;
			}
			
			if(wall[vx][vy  ].south == NOWALL && COST[vx  ][vy-1][0] > COST[vx][vy][0] + STRAIGHT ){//南に進める
				COST[vx][vy-1][0] = COST[vx][vy][0] + STRAIGHT;
				printf("南に行ける\n");
				index[vx][vy-1][0].x = vx;
				index[vx][vy-1][0].y = vy;
				index[vx][vy-1][0].dir = 0;
			}
			if(wall[vx][vy  ].east  == NOWALL && COST[vx  ][vy  ][1] > COST[vx][vy][0] + DIAGONAL ){//南東に進める
				COST[vx  ][vy][1] = COST[vx][vy][0] + DIAGONAL;
				printf("南東に行ける\n");
				index[vx  ][vy][1].x = vx;
				index[vx  ][vy][1].y = vy;
				index[vx  ][vy][1].dir = 0;
			}
			if(wall[vx][vy  ].west  == NOWALL && COST[vx-1][vy  ][1] > COST[vx][vy][0] + DIAGONAL ){//南西に進める
				COST[vx-1][vy][1] = COST[vx][vy][0] + DIAGONAL;
				printf("南西に行ける\n");
				index[vx-1][vy][1].x = vx;
				index[vx-1][vy][1].y = vy;
				index[vx-1][vy][1].dir = 0;
			}
		}else{//東
			if(wall[vx+1][vy].east  == NOWALL && COST[vx+1][vy  ][1] > COST[vx][vy][1] + STRAIGHT ){//東に進める
				COST[vx+1][vy][1] = COST[vx][vy][1] + STRAIGHT;
				printf("東に行ける %d\n",COST[vx+1][vy][1]);
				index[vx+1][vy][1] .x = vx;
				index[vx+1][vy][1] .y = vy;
				index[vx+1][vy][1] .dir = 1;
			}
			if(wall[vx+1][vy].north == NOWALL && COST[vx+1][vy  ][0] > COST[vx][vy][1] + DIAGONAL ){//北東に進める
				COST[vx+1][vy][0] = COST[vx][vy][1] + DIAGONAL;
				printf("北東に行ける %d\n",COST[vx+1][vy][0]);
				index[vx+1][vy][0] .x = vx;
				index[vx+1][vy][0] .y = vy;
				index[vx+1][vy][0] .dir = 1;
			}
			if(wall[vx+1][vy].south == NOWALL && COST[vx+1][vy-1][0] > COST[vx][vy][1] + DIAGONAL ){//南東に進める
				COST[vx+1][vy-1][0] = COST[vx][vy][1] + DIAGONAL;
				printf("南東に行ける %d\n",COST[vx+1][vy-1][0]);
				index[vx+1][vy-1][0] .x = vx;
				index[vx+1][vy-1][0] .y = vy;
				index[vx+1][vy-1][0] .dir = 1;
			}
			
			if(wall[vx  ][vy].west  == NOWALL && COST[vx-1][vy  ][1] > COST[vx][vy][1] + STRAIGHT ){//西に進める
				COST[vx-1][vy][1] = COST[vx][vy][1] + STRAIGHT;
				printf("西に行ける %d\n",COST[vx-1][vy][1]);
				index[vx-1][vy][1] .x = vx;
				index[vx-1][vy][1] .y = vy;
				index[vx-1][vy][1] .dir = 1;
			}
			if(wall[vx  ][vy].north == NOWALL && COST[vx  ][vy  ][0] > COST[vx][vy][1] + DIAGONAL ){//北西に進める
				COST[vx][vy][0] = COST[vx][vy][1] + DIAGONAL;
				printf("北西に行ける %d\n",COST[vx][vy][0]);
				index[vx][vy][0] .x = vx;
				index[vx][vy][0] .y = vy;
				index[vx][vy][0] .dir = 1;
			}
			if(wall[vx  ][vy].south == NOWALL && COST[vx  ][vy-1][0] > COST[vx][vy][1] + DIAGONAL ){//南西に進める
				COST[vx][vy-1][0] = COST[vx][vy][1] + DIAGONAL;
				printf("南西に行ける %d\n",COST[vx][vy-1][0]);
				index[vx][vy-1][0] .x = vx;
				index[vx][vy-1][0] .y = vy;
				index[vx][vy-1][0] .dir = 1;
			}
		}

	}
  
}

void setwall(void){
	//迷路の壁情報
	wall[0][0].north = NOWALL;
	wall[0][0].east  = WALL;
	wall[0][0].south = WALL;
	wall[0][0].west  = WALL;

	wall[1][0].north = NOWALL;
	wall[1][0].east  = NOWALL;
	wall[1][0].south = WALL;
	wall[1][0].west  = WALL;

	wall[2][0].north = WALL;
	wall[2][0].east  = NOWALL;
	wall[2][0].south = WALL;
	wall[2][0].west  = NOWALL;

	wall[3][0].north = NOWALL;
	wall[3][0].east  = WALL;
	wall[3][0].south = WALL;
	wall[3][0].west  = NOWALL;

	wall[0][1].north = NOWALL;
	wall[0][1].east  = NOWALL;
	wall[0][1].south = NOWALL;
	wall[0][1].west  = WALL;

	wall[1][1].north = NOWALL;
	wall[1][1].east  = NOWALL;
	wall[1][1].south = NOWALL;
	wall[1][1].west  = NOWALL;

	wall[2][1].north = WALL;
	wall[2][1].east  = WALL;
	wall[2][1].south = WALL;
	wall[2][1].west  = NOWALL;

	wall[3][1].north = NOWALL;
	wall[3][1].east  = WALL;
	wall[3][1].south = NOWALL;
	wall[3][1].west  = WALL;

	wall[0][2].north = NOWALL;
	wall[0][2].east  = WALL;
	wall[0][2].south = NOWALL;
	wall[0][2].west  = WALL;

	wall[1][2].north = WALL;
	wall[1][2].east  = NOWALL;
	wall[1][2].south = NOWALL;
	wall[1][2].west  = WALL;

	wall[2][2].north = NOWALL;
	wall[2][2].east  = NOWALL;
	wall[2][2].south = WALL;
	wall[2][2].west  = NOWALL;

	wall[3][2].north = WALL;
	wall[3][2].east  = WALL;
	wall[3][2].south = NOWALL;
	wall[3][2].west  = NOWALL;

	wall[0][3].north = WALL;
	wall[0][3].east  = NOWALL;
	wall[0][3].south = NOWALL;
	wall[0][3].west  = WALL;

	wall[1][3].north = WALL;
	wall[1][3].east  = NOWALL;
	wall[1][3].south = WALL;
	wall[1][3].west  = NOWALL;

	wall[2][3].north = WALL;
	wall[2][3].east  = NOWALL;
	wall[2][3].south = NOWALL;
	wall[2][3].west  = NOWALL;

	wall[3][3].north = WALL;
	wall[3][3].east  = WALL;
	wall[3][3].south = WALL;
	wall[3][3].west  = NOWALL;
}
  
