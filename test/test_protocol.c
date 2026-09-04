/* Banc d'essai hote : compile le VRAI protocol.c et le VRAI pid.c. */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "config.h"
#include "robot.h"
#include "encoders.h"
#include "motors.h"
#include "bno055.h"
#include "testmodes.h"
#include "protocol.h"
#include "pico/stdlib.h"

/* ---- environnement simule ---- */
uint64_t h_now_us = 1000000;
static const char *g_in; static size_t g_in_pos;
int h_getchar(void){ return g_in[g_in_pos] ? (unsigned char)g_in[g_in_pos++] : PICO_ERROR_TIMEOUT; }

static char g_tx[512]; static size_t g_tx_len;
void h_put(const char *s,int len,bool nl,bool cr){ (void)cr;
    memcpy(g_tx+g_tx_len,s,(size_t)len); g_tx_len+=(size_t)len;
    if(nl) { g_tx[g_tx_len++]='\n'; }
    g_tx[g_tx_len]=0; }

static char g_log[4096]; static size_t g_log_len;
void h_log(const char *s,size_t n){ if(g_log_len+n<sizeof(g_log)){memcpy(g_log+g_log_len,s,n); g_log_len+=n; g_log[g_log_len]=0;} }

/* ---- bouchons des modules non testes ici ---- */
robot_t g_robot;
static encoders_t g_enc; static bno055_t g_imu;
static int g_reset_calls, g_test_mode_arg, g_failsafe_calls;
const encoders_t *encoders_state(void){ return &g_enc; }
void encoders_reset(void){ g_reset_calls++; }
void encoders_update(float d){(void)d;}
void encoders_init(void){}
int32_t encoders_raw(int s){(void)s; return 0;}
const bno055_t *bno055_state(void){ return &g_imu; }
void motors_coast_all(void){}
void motors_set(int s,int d){(void)s;(void)d;}
int  motors_get(int s){(void)s; return 0;}
void testmodes_enter(int m){ g_test_mode_arg=m; g_robot.test_mode=m; }
void robot_failsafe(void){ g_failsafe_calls++; g_robot.ctrl_mode=CTRL_STOP; }

/* ---- utilitaires ---- */
static int fails=0, total=0;
static void feed(const char *s){ g_in=s; g_in_pos=0; protocol_poll(); }
static void ck(const char *what,int cond){ total++;
    if(!cond){fails++; printf("  ECHEC %s\n",what);} else printf("  OK    %s\n",what); }

/* Verificateur independant du checksum, transcrit depuis la documentation
   destinee au Pi (protocol.h) — et non depuis le code emetteur. */
static int verify_frame_like_the_pi(const char *line)
{
    char buf[512]; snprintf(buf,sizeof buf,"%s",line);
    char *nl=strchr(buf,'\n'); if(!nl) return 0; *nl=0;
    char *sp=strrchr(buf,' '); if(!sp) return 0;
    *sp=0; const char *chk_s=sp+1;
    uint8_t x=0; for(const char *p=buf;*p;p++) x^=(uint8_t)*p;
    return (int)strtol(chk_s,NULL,16)==(int)x;
}

int main(void)
{
    memset(&g_robot,0,sizeof g_robot);
    for(int i=0;i<2;i++) pid_init(&g_robot.pid[i],PID_KP_DEFAULT,PID_KI_DEFAULT,
                                  PID_KD_DEFAULT,PID_KFF,-1000.f,1000.f);
    protocol_init();

    puts("Commandes valides :");
    feed("v 1200 -800\n");
    ck("v : consignes et passage en asservissement",
       g_robot.sp_ticks[0]==1200 && g_robot.sp_ticks[1]==-800 && g_robot.ctrl_mode==CTRL_VELOCITY);
    feed("o -1000 1000\n");
    ck("o : boucle ouverte aux bornes",
       g_robot.ol_duty[0]==-1000 && g_robot.ol_duty[1]==1000 && g_robot.ctrl_mode==CTRL_OPENLOOP);
    feed("o -5000 5000\n");
    ck("o : ecretage a +/-DUTY_MAX", g_robot.ol_duty[0]==-1000 && g_robot.ol_duty[1]==1000);
    int fs=g_failsafe_calls; feed("s\n");
    ck("s : arret immediat", g_failsafe_calls==fs+1);
    int rc=g_reset_calls; feed("r\n");
    ck("r : remise a zero des compteurs", g_reset_calls==rc+1);
    feed("k 0.25 1.5 0.01\n");
    ck("k : gains appliques aux deux roues",
       g_robot.pid[0].kp==0.25f && g_robot.pid[1].ki==1.5f && g_robot.pid[0].kd==0.01f);
    feed("t 3\n");   ck("t 3 : mode de test transmis", g_test_mode_arg==3);
    feed("t 0\n");   ck("t 0 : retour au mode normal", g_test_mode_arg==0);
    feed("v -2147483648 2147483647\n");
    ck("v : pas de limite de vitesse (int32 complet)",
       g_robot.sp_ticks[0]==INT32_MIN && g_robot.sp_ticks[1]==INT32_MAX);

    puts("\nTolerance de format :");
    feed("  v   10   20  \n");
    ck("espaces multiples et en tete", g_robot.sp_ticks[0]==10 && g_robot.sp_ticks[1]==20);
    feed("v 30 40\r\n");
    ck("terminaison CRLF", g_robot.sp_ticks[0]==30 && g_robot.sp_ticks[1]==40);
    feed("\n\n\nv 50 60\n");
    ck("lignes vides ignorees", g_robot.sp_ticks[0]==50);
    feed("v 70 80\nv 90 100\nv 110 120\n");
    ck("plusieurs commandes dans un meme paquet", g_robot.sp_ticks[0]==110);

    puts("\nRejet des commandes invalides (l'etat ne doit pas bouger) :");
    struct { const char *cmd, *why; } bad[] = {
        {"v 1\n","argument manquant"}, {"v\n","aucun argument"},
        {"v 1 2 3\n","argument surnumeraire"}, {"v a b\n","non numerique"},
        {"v 1 2x\n","suffixe parasite"}, {"z 1 2\n","commande inconnue"},
        {"s 5\n","argument sur une commande sans argument"},
        {"r 1\n","argument sur r"}, {"k 1 2\n","gain manquant"},
        {"t 9\n","mode de test hors plage"}, {"t -1\n","mode negatif"},
        {"t\n","mode manquant"}, {"o 1\n","une seule roue"},
    };
    for(unsigned i=0;i<sizeof bad/sizeof*bad;i++){
        g_robot.sp_ticks[0]=4242; g_robot.sp_ticks[1]=4242;
        g_robot.ol_duty[0]=77; uint32_t nb=g_robot.rx_bad;
        int tm=g_test_mode_arg;
        feed(bad[i].cmd);
        char lbl[128]; snprintf(lbl,sizeof lbl,"\"%.*s\" rejete (%s)",
            (int)strcspn(bad[i].cmd,"\r\n"), bad[i].cmd, bad[i].why);
        ck(lbl, g_robot.rx_bad==nb+1 && g_robot.sp_ticks[0]==4242
                && g_robot.ol_duty[0]==77 && g_test_mode_arg==tm);
    }

    puts("\nRobustesse de la reception :");
    { char big[RX_LINE_MAX+80]; memset(big,'9',sizeof big-2);
      big[0]='v'; big[sizeof big-2]='\n'; big[sizeof big-1]=0;
      uint32_t nb=g_robot.rx_bad; g_robot.sp_ticks[0]=1;
      feed(big);
      ck("ligne plus longue que RX_LINE_MAX : purgee, pas de debordement",
         g_robot.rx_bad==nb+1 && g_robot.sp_ticks[0]==1); }
    feed("v 1 2\n"); ck("resynchronisation apres ligne trop longue",
                        g_robot.sp_ticks[0]==1 && g_robot.sp_ticks[1]==2);

    puts("\nChien de garde : toute commande valide le rearme :");
    g_robot.link_lost=true; h_now_us=5000000; feed("v 1 1\n");
    ck("commande valide : lien declare retabli",
       !g_robot.link_lost && g_robot.last_cmd_us==5000000);
    g_robot.link_lost=true; h_now_us=6000000; g_robot.last_cmd_us=0;
    feed("bidon\n");
    ck("commande invalide : chien de garde NON rearme",
       g_robot.link_lost && g_robot.last_cmd_us==0);

    puts("\nTrame d'etat et checksum :");
    struct { int32_t el,er,vl,vr; int16_t g[3],a[3]; uint32_t t; } cases[] = {
        {0,0,0,0,{0,0,0},{0,0,0},0},
        {1496,-1496,4362,-4362,{12,-34,56},{-78,910,-1011},123456},
        {INT32_MAX,INT32_MIN,INT32_MAX,INT32_MIN,{32767,-32768,0},{32767,-32768,1},4294967295u},
    };
    for(unsigned i=0;i<sizeof cases/sizeof*cases;i++){
        g_enc.count[0]=cases[i].el; g_enc.count[1]=cases[i].er;
        g_enc.vel_i[0]=cases[i].vl; g_enc.vel_i[1]=cases[i].vr;
        memcpy(g_imu.gyro,cases[i].g,sizeof g_imu.gyro);
        memcpy(g_imu.accel,cases[i].a,sizeof g_imu.accel);
        g_tx_len=0; protocol_send_state(cases[i].t);
        char lbl[256]; snprintf(lbl,sizeof lbl,"checksum verifiable cote Pi : %.*s",
                                (int)strcspn(g_tx,"\n"), g_tx);
        ck(lbl, verify_frame_like_the_pi(g_tx));
        total++; if(g_tx[g_tx_len-1]!='\n'){fails++; puts("  ECHEC terminaison \\n");}
        else puts("  OK    trame terminee par un seul \\n, sans \\r");
        total++; if(strchr(g_tx,'\r')){fails++; puts("  ECHEC un \\r s'est glisse dans la trame");}
        else puts("  OK    aucun \\r dans la trame");
    }

    /* Un octet fausse doit invalider le checksum : preuve que le controle sert. */
    g_tx_len=0; protocol_send_state(999); g_tx[3]= g_tx[3]=='7'?'8':'7';
    ck("un octet altere invalide bien le checksum", !verify_frame_like_the_pi(g_tx));

    printf("\n%d cas, %d echec(s)\n", total, fails);
    return fails!=0;
}
