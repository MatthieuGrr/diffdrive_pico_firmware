/* Banc d'essai hote : limitation du courant d'appel (ilim.h) et
   pid_update_clamped. */
#include <stdio.h>
#include <math.h>
#include "ilim.h"
#include "pid.h"
static int fails=0,total=0;
static void ck(const char *w,int c){ total++; if(!c){fails++;printf("  ECHEC %s\n",w);} else printf("  OK    %s\n",w); }
int main(void){
    ilim_win_t w;
    puts("Fenetre de courant :");
    w=ilim_window(0,ILIM_VREF_MV);
    ck("a l'arret, pleine puissance retenue a +/-DELTA", ilim_apply(1000,w)==ILIM_DUTY_DELTA && ilim_apply(-1000,w)==-ILIM_DUTY_DELTA);
    ck("a l'arret, petite consigne inchangee", ilim_apply(ILIM_DUTY_DELTA-1,w)==ILIM_DUTY_DELTA-1);
    ck("duty nul (frein) toujours accepte", ilim_apply(0,ilim_window(6500,ILIM_VREF_MV))==0);
    w=ilim_window(6500,ILIM_VREF_MV);
    ck("a pleine vitesse, pleine puissance libre", ilim_apply(1000,w)==1000);
    ck("a pleine vitesse, inversion brutale retenue", ilim_apply(-1000,w)==1000-ILIM_DUTY_DELTA);
    w=ilim_window(3250,ILIM_VREF_MV);
    ck("consigne qui suit le moteur (rampe de l'hote) jamais modifiee", ilim_apply(600,w)==600 && ilim_apply(400,w)==400);
    w=ilim_window(3250,ILIM_VREF_MV/2);
    ck("batterie a moitie : f.c.e.m. en duty doublee", ilim_apply(1000,w)==1000);
    w=ilim_window(3250,0);
    ck("sans mesure de tension : tension nominale", ilim_apply(1000,w)==(int)(500+ILIM_DUTY_DELTA) || ILIM_DUTY_DELTA>=500);
    puts("\nPID borne par la fenetre :");
    pid_t p; pid_init(&p,0.1f,0.5f,0,0,-1000,1000);
    float o=0; for(int i=0;i<50;i++) o=pid_update_clamped(&p,6000,0,0.02f,-200,200);
    ck("sortie retenue a la borne haute", fabsf(o-200)<1e-3);
    ck("integrale gelee pendant la retenue", p.integ==0.0f);
    o=pid_update_clamped(&p,6000,0,0.02f,-1000,1000);
    ck("borne relachee : la sortie repart sans a-coup d'integrale", o<=0.1f*6000+0.5f*120+1);
    printf("\n%d cas, %d echec(s)\n",total,fails);
    return fails!=0;
}
