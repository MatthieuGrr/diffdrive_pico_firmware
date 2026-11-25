/* Banc d'essai hote du VRAI motors.c : on simule le compteur PWM pas a pas
   et on applique la table de verite du MDD10A (signe-amplitude) a chaque
   pas, pour mesurer la fraction d'entrainement reellement obtenue et l'etat
   du pont pendant le temps mort. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "config.h"
#include "motors.h"

uint16_t h_lvl[8][2];
int h_gpio[48];
void h_log(const char *s,size_t n){(void)s;(void)n;}   /* LOG() muet */

static int fails=0,total=0;
static void ck(const char *w,int c){ total++; if(!c){fails++;printf("  ECHEC %s\n",w);} else printf("  OK    %s\n",w); }

/* Table de verite du MDD10A : +1 sortie A haute, -1 sortie B haute,
   2 frein (A = B = 0). Il n'existe pas d'etat roue libre. */
static int bridge(int pwm,int dir){ if(!pwm) return 2; return dir ? -1 : 1; }

typedef struct { float drive; int brake_steps; } profile_t;

static const unsigned PWM_PIN[2]={PIN_MOT_L_PWM,PIN_MOT_R_PWM};
static const unsigned DIR_PIN[2]={PIN_MOT_L_DIR,PIN_MOT_R_DIR};

/* Parcourt un cycle PWM complet et resume ce que voit le moteur m. */
static profile_t profile(int m, unsigned span)
{
    profile_t r={0,0}; int fwd=0,rev=0;
    unsigned sl=(PWM_PIN[m]>>1)&7u, ch=PWM_PIN[m]&1u;
    for(unsigned t=0;t<span;t++){
        int pwm = t < h_lvl[sl][ch];      /* sortie haute tant que compteur < niveau */
        switch(bridge(pwm,h_gpio[DIR_PIN[m]])){ case 1:fwd++;break; case -1:rev++;break;
                                               default:r.brake_steps++; }
    }
    r.drive = (float)(fwd-rev)/(float)span;
    return r;
}

int main(void)
{
    motors_init();
    unsigned span = (unsigned)motors_pwm_top()+1u;
    printf("MDD10A, signe-amplitude | span PWM = %u pas\n\n", span);
    ck("span PWM = 7500 pas a 150 MHz / 20 kHz (limite du MDD10A)", span==7500u);

    const int inv[2]={ MOT_L_INVERT, MOT_R_INVERT };
    const char *nm[2]={"G","D"};

    puts("\nPolarite et amplitude sur toute la plage :");
    for(int m=0;m<2;m++){
        for(int duty=-1000; duty<=1000; duty+=1){
            motors_set(m,duty);
            profile_t p = profile(m, span);
            float want = (float)duty/1000.0f * (inv[m]?-1.f:1.f);
            if(fabsf(p.drive-want) > 1.0f/(float)span + 1e-6f){
                printf("  ECHEC roue %s duty=%+d : entrainement %.4f, attendu %.4f\n",
                       nm[m],duty,p.drive,want); fails++; total++; goto next;
            }
        }
        total++;
        printf("  OK    roue %s : entrainement conforme sur les 2001 consignes%s\n",
               nm[m], inv[m]?" (inversion prise en compte)":"");
        next:;
    }

    puts("\nTemps mort et arret :");
    motors_set(MOT_L,300);
    profile_t p = profile(MOT_L,span);
    ck("duty 300 : 70 % du cycle en freinage (decroissance lente)",
       abs(p.brake_steps - (int)(span*7/10)) <= 1);

    const float sL = MOT_L_INVERT ? -1.0f : 1.0f;
    motors_set(MOT_L,0);
    p = profile(MOT_L,span);
    ck("duty 0 : PWM a zero, moteur freine sur tout le cycle", p.brake_steps==(int)span);
    motors_set(MOT_L,1000);
    p = profile(MOT_L,span);
    ck("duty +1000 : entrainement plein, sans temps mort", p.drive==sL);
    motors_set(MOT_L,-1000);
    p = profile(MOT_L,span);
    ck("duty -1000 : entrainement plein en sens oppose", p.drive==-sL);
    motors_set(MOT_L,5000);
    p = profile(MOT_L,span);
    ck("consigne hors plage ecretee a +1000", p.drive==sL);
    motors_set(MOT_L,-5000);
    p = profile(MOT_L,span);
    ck("consigne hors plage ecretee a -1000", p.drive==-sL);

    motors_set(MOT_L,700); motors_set(MOT_R,-400);
    motors_stop_all();
    ck("motors_stop_all : les deux PWM a zero (freinage)",
       profile(MOT_L,span).brake_steps==(int)span && profile(MOT_R,span).brake_steps==(int)span);
    ck("motors_get() remis a zero apres arret",
       motors_get(MOT_L)==0 && motors_get(MOT_R)==0);

    puts("\nInhibition de securite (alerte de surintensite) :");
    motors_set(MOT_L,800); motors_set(MOT_R,-800);
    motors_inhibit(true);
    ck("motors_inhibit(true) freine immediatement les deux moteurs",
       profile(MOT_L,span).brake_steps==(int)span && profile(MOT_R,span).brake_steps==(int)span);
    motors_set(MOT_L,900); motors_set(MOT_R,900);
    ck("consignes ignorees tant que l'inhibition dure",
       profile(MOT_L,span).brake_steps==(int)span && motors_get(MOT_L)==0 && motors_inhibited());
    motors_inhibit(false);
    motors_set(MOT_L,900);
    ck("inhibition levee : les consignes reprennent effet",
       fabsf(profile(MOT_L,span).drive - 0.9f*sL) < 2.0f/(float)span);

    puts("\nMonotonie (aucune inversion ni discontinuite dans la rampe) :");
    float prev=-2.f; int mono=1;
    for(int duty=-1000; duty<=1000; duty++){
        motors_set(MOT_L,duty);
        float d = profile(MOT_L,span).drive * sL;
        if(d < prev - 1e-6f) mono=0;
        prev=d;
    }
    ck("entrainement monotone de -1000 a +1000 (roue G, inversion comprise)", mono);

    printf("\n%d cas, %d echec(s)\n",total,fails);
    return fails!=0;
}
