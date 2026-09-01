/* Banc d'essai hote du VRAI pid.c. Plant de premier ordre saturant, qui
   reproduit grossierement une roue : vitesse = gain * duty, avec constante
   de temps, et duty physiquement borne a +/-1000. */
#include <stdio.h>
#include <math.h>
#include "config.h"
#include "pid.h"

static int fails=0,total=0;
static void ck(const char *w,int c){ total++; if(!c){fails++;printf("  ECHEC %s\n",w);} else printf("  OK    %s\n",w); }

#define DT 0.02f
typedef struct { float v, gain, tau; } plant_t;
static float plant_step(plant_t *p,float duty){
    float target = p->gain*duty;
    p->v += (target-p->v)*(DT/p->tau);
    return p->v;
}

int main(void)
{
    puts("Regime etabli :");
    { pid_t c; plant_t p={0,4.4f,0.15f};   /* 1000 duty -> 4400 ticks/s */
      pid_init(&c,PID_KP_DEFAULT,PID_KI_DEFAULT,PID_KD_DEFAULT,PID_KFF,-1000,1000);
      float m=0; for(int i=0;i<400;i++) m=plant_step(&p,pid_update(&c,2000.f,m,DT));
      printf("        consigne 2000 -> mesure %.1f ticks/s\n",(double)m);
      ck("l'integrateur annule l'erreur statique", fabsf(m-2000.f)<5.f); }

    puts("\nSortie bornee a la plage physique :");
    { pid_t c; plant_t p={0,4.4f,0.15f};
      pid_init(&c,PID_KP_DEFAULT,PID_KI_DEFAULT,PID_KD_DEFAULT,PID_KFF,-1000,1000);
      float m=0,mx=0,mn=0;
      for(int i=0;i<300;i++){ float u=pid_update(&c,100000.f,m,DT);
        if(u>mx)mx=u;
        if(u<mn)mn=u;
        m=plant_step(&p,u); }
      ck("jamais au-dela de +DUTY_MAX", mx<=1000.f);
      for(int i=0;i<300;i++){ float u=pid_update(&c,-100000.f,m,DT);
        if(u<mn)mn=u;
        m=plant_step(&p,u); }
      ck("jamais en deca de -DUTY_MAX", mn>=-1000.f); }

    puts("\nAnti-emballement — consigne inatteignable maintenue 10 s :");
    /* Signature de l'emballement : apres une longue saturation, l'integrale
       gonflee maintient la sortie en butee bien apres la baisse de consigne.
       On mesure donc le temps de RETOUR a une consigne atteignable. */
    float t_pid = -1.f, t_naif = -1.f;
    { pid_t c; plant_t p={0,4.4f,0.15f};
      pid_init(&c,PID_KP_DEFAULT,PID_KI_DEFAULT,PID_KD_DEFAULT,PID_KFF,-1000,1000);
      float m=0;
      for(int i=0;i<500;i++) m=plant_step(&p,pid_update(&c,50000.f,m,DT));
      float i_term = c.ki*c.integ;
      printf("        contribution integrale apres saturation : %.1f\n",(double)i_term);
      ck("l'integrale reste dans la plage de sortie", fabsf(i_term)<=1000.f+1e-3f);
      int back=-1;
      for(int i=0;i<2000;i++){ m=plant_step(&p,pid_update(&c,1000.f,m,DT));
        if(back<0 && fabsf(m-1000.f)<50.f) back=i; }
      t_pid = back<0 ? -1.f : back*DT;
      printf("        retour a 1000 ticks/s en %.2f s\n",(double)t_pid);
      ck("la consigne atteignable est effectivement rejointe", fabsf(m-1000.f)<5.f); }

    { /* Temoin : meme scenario, integration inconditionnelle. */
      plant_t p={0,4.4f,0.15f};
      float m=0, integ=0, kp=PID_KP_DEFAULT, ki=PID_KI_DEFAULT;
      for(int i=0;i<500;i++){ float e=50000.f-m; integ+=e*DT;
        float u=kp*e+ki*integ; if(u>1000.f)u=1000.f; if(u<-1000.f)u=-1000.f;
        m=plant_step(&p,u); }
      int back=-1;
      for(int i=0;i<2000;i++){ float e=1000.f-m; integ+=e*DT;
        float u=kp*e+ki*integ; if(u>1000.f)u=1000.f; if(u<-1000.f)u=-1000.f;
        m=plant_step(&p,u); if(back<0 && fabsf(m-1000.f)<50.f) back=i; }
      t_naif = back<0 ? -1.f : back*DT;
      if(t_naif<0) printf("        temoin naif : toujours pas revenu apres 40 s\n");
      else         printf("        temoin naif : retour en %.2f s\n",(double)t_naif); }
    ck("l'anti-emballement revient, l'integrateur naif reste bloque en butee",
       t_pid>=0.f && t_naif<0.f);

    puts("\nAnti-emballement en sens inverse :");
    { pid_t c; plant_t p={0,4.4f,0.15f};
      pid_init(&c,PID_KP_DEFAULT,PID_KI_DEFAULT,PID_KD_DEFAULT,PID_KFF,-1000,1000);
      float m=0;
      for(int i=0;i<500;i++) m=plant_step(&p,pid_update(&c,-50000.f,m,DT));
      ck("integrale bornee aussi en negatif", fabsf(c.ki*c.integ)<=1000.f+1e-3f);
      int back=-1;
      for(int i=0;i<2000;i++){ m=plant_step(&p,pid_update(&c,-1000.f,m,DT));
        if(back<0 && fabsf(m+1000.f)<50.f) back=i; }
      printf("        retour a -1000 ticks/s en %.2f s\n",back<0?-1.0:back*(double)DT);
      ck("retour effectif a la consigne negative", back>=0 && fabsf(m+1000.f)<5.f); }

    puts("\nDerive calculee sur la mesure, pas sur l'erreur :");
    { pid_t a,b;
      pid_init(&a,0.1f,0.f,5.0f,0.f,-1000,1000);
      pid_init(&b,0.1f,0.f,5.0f,0.f,-1000,1000);
      pid_update(&a,0.f,0.f,DT); pid_update(&b,0.f,0.f,DT);
      float step = pid_update(&a,1000.f,0.f,DT);   /* echelon de consigne */
      float meas = pid_update(&b,0.f,-1000.f,DT);  /* meme erreur, via la mesure */
      printf("        echelon de consigne -> %.1f | echelon de mesure -> %.1f\n",
             (double)step,(double)meas);
      ck("un echelon de consigne ne produit pas de pic de derivee", fabsf(step)<=100.1f);
      ck("un echelon de mesure, lui, sollicite bien la derivee", fabsf(meas)>1000.f-1.f); }

    puts("\nReglage des gains a chaud :");
    { pid_t c; pid_init(&c,0.1f,10.f,0.f,0.f,-1000,1000);
      float m=0; for(int i=0;i<100;i++) pid_update(&c,500.f,m,DT);
      pid_set_gains(&c,0.1f,1000.f,0.f);           /* ki multiplie par 100 */
      pid_update(&c,500.f,m,DT);
      ck("l'integrale est re-ecretee avec le nouveau ki",
         fabsf(c.ki*c.integ)<=1000.f+1e-3f);
      ck("la sortie reste dans la plage", fabsf(c.last_out)<=1000.f); }

    puts("\nCas degeneres :");
    { pid_t c; pid_init(&c,0.1f,0.5f,0.01f,0.f,-1000,1000);
      float u1=pid_update(&c,100.f,0.f,DT);
      float u2=pid_update(&c,100.f,0.f,0.f);       /* dt nul */
      ck("dt nul : renvoie la derniere sortie, sans division par zero", u1==u2);
      pid_reset(&c);
      ck("pid_reset remet integrale et etat a zero", c.integ==0.f && !c.primed);
      pid_init(&c,0.1f,0.f,0.f,0.f,-1000,1000);
      for(int i=0;i<50;i++) pid_update(&c,500.f,0.f,DT);
      ck("ki nul : aucune accumulation", c.integ==0.f); }

    printf("\n%d cas, %d echec(s)\n",total,fails);
    return fails!=0;
}
