/* Banc d'essai hote du VRAI motors.c : on simule le compteur PWM pas a pas
   et on classe l'etat du pont DRV8871 a chaque pas, pour mesurer la fraction
   d'entrainement reellement obtenue et le comportement pendant le temps mort. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"
#include "motors.h"

uint16_t h_lvl[8][2];
void h_log(const char *s,size_t n){(void)s;(void)n;}   /* LOG() muet */

static int fails=0,total=0;
static void ck(const char *w,int c){ total++; if(!c){fails++;printf("  ECHEC %s\n",w);} else printf("  OK    %s\n",w); }

/* Table de verite du DRV8871 : renvoie +1 avant, -1 arriere, 0 roue libre, 2 frein */
static int bridge(int in1,int in2){ if(in1&&!in2) return 1; if(!in1&&in2) return -1;
                                    if(!in1&&!in2) return 0;
                                    return 2; }

typedef struct { float drive; int dead_state; int mixed_dead; } profile_t;

/* Parcourt un cycle PWM complet et resume ce que voit le moteur. */
static profile_t profile(unsigned slice, unsigned c1, unsigned c2, unsigned span)
{
    profile_t r={0,0,0}; int fwd=0,rev=0,coast=0,brake=0;
    for(unsigned t=0;t<span;t++){
        int in1 = t < h_lvl[slice][c1];      /* sortie haute tant que compteur < niveau */
        int in2 = t < h_lvl[slice][c2];
        switch(bridge(in1,in2)){ case 1:fwd++;break; case -1:rev++;break;
                                 case 0:coast++;break; default:brake++; }
    }
    r.drive = (float)(fwd-rev)/(float)span;
    if(coast&&brake) r.mixed_dead=1;
    r.dead_state = brake?2:0;
    return r;
}

int main(void)
{
    motors_init();
    unsigned span = (unsigned)motors_pwm_top()+1u;
    printf("Mode de decroissance compile : %s | span PWM = %u pas\n\n",
           motors_decay_name(), span);
    ck("span PWM = 6000 pas a 150 MHz / 25 kHz", span==6000u);

    const unsigned sl[2]={ (PIN_MOT_L_IN1>>1)&7u, (PIN_MOT_R_IN1>>1)&7u };
    const int inv[2]={ MOT_L_INVERT, MOT_R_INVERT };
    const char *nm[2]={"G","D"};

    puts("\nPolarite et amplitude sur toute la plage :");
    for(int m=0;m<2;m++){
        for(int duty=-1000; duty<=1000; duty+=1){
            motors_set(m,duty);
            profile_t p = profile(sl[m], PIN_MOT_L_IN1&1u, PIN_MOT_L_IN2&1u, span);
            float want = (float)duty/1000.0f * (inv[m]?-1.f:1.f);
            if(fabsf(p.drive-want) > 1.0f/(float)span + 1e-6f){
                printf("  ECHEC roue %s duty=%+d : entrainement %.4f, attendu %.4f\n",
                       nm[m],duty,p.drive,want); fails++; total++; goto next;
            }
            if(duty!=0 && p.mixed_dead){
                printf("  ECHEC roue %s duty=%+d : temps mort incoherent\n",nm[m],duty);
                fails++; total++; goto next;
            }
        }
        total++;
        printf("  OK    roue %s : entrainement conforme sur les 2001 consignes%s\n",
               nm[m], inv[m]?" (inversion prise en compte)":"");
        next:;
    }

    puts("\nNature du temps mort (c'est cela que MOTOR_DECAY_MODE change) :");
    motors_set(MOT_L,300);
    profile_t p = profile(sl[0],PIN_MOT_L_IN1&1u,PIN_MOT_L_IN2&1u,span);
#if MOTOR_DECAY_MODE == DECAY_SLOW
    ck("decroissance lente : temps mort en FREINAGE (courant maintenu)", p.dead_state==2);
#else
    ck("decroissance rapide : temps mort en ROUE LIBRE", p.dead_state==0);
#endif

    puts("\nCas limites :");
    motors_set(MOT_L,0);
    ck("duty 0 : les deux entrees au niveau bas (roue libre)",
       h_lvl[sl[0]][0]==0 && h_lvl[sl[0]][1]==0);
    motors_set(MOT_L,1000);
    p = profile(sl[0],0,1,span);
    ck("duty +1000 : entrainement plein avant, sans temps mort", p.drive==1.0f);
    motors_set(MOT_L,-1000);
    p = profile(sl[0],0,1,span);
    ck("duty -1000 : entrainement plein arriere", p.drive==-1.0f);
    motors_set(MOT_L,5000);
    p = profile(sl[0],0,1,span);
    ck("consigne hors plage ecretee a +1000", p.drive==1.0f);
    motors_set(MOT_L,-5000);
    p = profile(sl[0],0,1,span);
    ck("consigne hors plage ecretee a -1000", p.drive==-1.0f);

    motors_set(MOT_L,700); motors_set(MOT_R,-400);
    motors_coast_all();
    ck("coupure de securite : les 4 entrees au niveau bas, quel que soit le mode",
       h_lvl[sl[0]][0]==0 && h_lvl[sl[0]][1]==0 &&
       h_lvl[sl[1]][0]==0 && h_lvl[sl[1]][1]==0);
    ck("motors_get() remis a zero apres coupure",
       motors_get(MOT_L)==0 && motors_get(MOT_R)==0);

    puts("\nMonotonie (aucune inversion ni discontinuite dans la rampe) :");
    float prev=-2.f; int mono=1;
    for(int duty=-1000; duty<=1000; duty++){
        motors_set(MOT_L,duty);
        profile_t q = profile(sl[0],0,1,span);
        if(q.drive < prev - 1e-6f) mono=0;
        prev=q.drive;
    }
    ck("entrainement strictement croissant de -1000 a +1000 (roue G)", mono);

    printf("\n%d cas, %d echec(s)\n",total,fails);
    return fails!=0;
}
