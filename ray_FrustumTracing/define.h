#ifndef _DEFINE_H
#define _DEFINE_H

/*definicje pól do mapy świata*/
#define ENEMY        6
#define ENVIRONMENT  5
#define SCARP        4
#define NOTPASS      1
#define PASS         0

/*define'y dla wyszukiwania drogi*/
#define BEVELMOVE 14
#define STRAIGHTMOVE 10
/*define'y do poruszania się po terenie, jak wywołujemy dla czegoś co się porusza po terenie(zombie)
  to podajemy do getHeight WALKER, a jeśli używamy getHeight tylko dla testów (np czy pocisk uderzył w ziemie) to NOWALKER*/
#define WALKER true
#define NOWALKER false

/*define'y dla pola attitude w ComputerChar*/
#define ENEMYCHAR true
#define NEUTRALCHAR false

#define PI 3.14159265

#define ANIMATION_OFF -1
#define MAX_FRAME 200
#define IDLE -1
#define WALK 0
#define FIGHT 1
#define DIE 2

#define ENEMY_ATTACK_SPEED 30
#define SAME_TEXTURE 100
#define ALL 101


#define DEAD_COMP_CHAR false
#define ALIVE_COMP_CHAR true
#define DEAD false
#define ALIVE true

/*WYLECI W CHUUUUUUJ*/
//#define BULLET_DAMAGE 20
//#define MAX_MAG_COUNT 5
//#define BULLET_PER_MAG_COUNT 30
//#define MAX_WEAPON_RANGE 30
#define WEAPON_INTERVAL 1
//#define WEAPON_RELOAD_TIME 20
/*to juz nie*/

#define SHOOT_FORBIEDEN false
#define SHOOT_ALLOWED true
#define IS_AMMO true
#define NO_AMMO false
#define IS_RELOAD true
#define NO_RELOAD false


#endif
