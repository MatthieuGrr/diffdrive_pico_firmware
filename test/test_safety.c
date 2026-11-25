/* Banc d'essai hote : compile le VRAI safety.c, et les conversions de
   ina226.h (fonctions pures, sans acces materiel). */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "safety.h"
#include "ina226.h"

static int fails=0,total=0;
static void ck(const char *w,int c){ total++; if(!c){fails++;printf("  ECHEC %s\n",w);} else printf("  OK    %s\n",w); }

static safety_in_t nominal(void){
    safety_in_t in={ .ina_present=true, .vbat_mv=19000, .ibat_ma=1200,
                     .alert_fired=false, .alert_pin_active=false };
    return in;
}
/* Fait tourner la logique n periodes de 20 ms. */
static bool run(safety_t *s, const safety_in_t *in, int n){
    bool inh=false; for(int i=0;i<n;i++) inh=safety_update(s,in,20); return inh;
}

int main(void)
{
    puts("Conversions INA226 (shunt 10 mOhm) :");
    ck("shunt brut 4000 (10 mV) -> 1000 mA", ina226_ma_from_raw(4000,10000)==1000);
    ck("shunt brut -4000 -> -1000 mA (courant de recuperation)", ina226_ma_from_raw(-4000,10000)==-1000);
    ck("shunt brut 32767 -> 8192 mA (pleine echelle)", ina226_ma_from_raw(32767,10000)==8192);
    ck("shunt brut 1 -> 0 mA (0,25 mA arrondi)", ina226_ma_from_raw(1,10000)==0);
    ck("shunt brut 2 -> 1 mA (0,5 mA arrondi au plus proche)", ina226_ma_from_raw(2,10000)==1);
    ck("bus brut 16000 -> 20000 mV (LSB 1,25 mV)", ina226_mv_from_raw(16000)==20000);
    ck("bus brut 0xFFFF -> 81919 mV, sans debordement", ina226_mv_from_raw(0xFFFF)==81919);
    ck("seuil 7500 mA -> limite d'alerte 30000", ina226_alert_limit_from_ma(7500,10000)==30000);
    ck("seuil hors echelle ecrete a 32767", ina226_alert_limit_from_ma(20000,10000)==32767);
    ck("limite d'alerte coherente avec la conversion inverse",
       ina226_ma_from_raw((int16_t)ina226_alert_limit_from_ma(OC_TRIP_MA,10000),10000)==OC_TRIP_MA);

    safety_t s; safety_in_t in;

    puts("\nFonctionnement nominal :");
    safety_init(&s); in=nominal();
    ck("batterie saine, courant normal : moteurs autorises", !run(&s,&in,100));
    ck("aucun drapeau", safety_flags(&s,&in)==0u);

    puts("\nSurintensite :");
    safety_init(&s); in=nominal(); in.alert_fired=true; in.alert_pin_active=true;
    ck("ALERT materielle : verrouillage immediat", run(&s,&in,1) && (s.latched & FLAG_OVERCURRENT));
    in.alert_fired=false;
    ck("le defaut reste verrouille une fois l'alerte retombee", run(&s,&in,50));
    safety_try_clear(&s,&in);
    ck("'s' refuse tant que la broche ALERT est basse", s.latched & FLAG_OVERCURRENT);
    in.alert_pin_active=false;
    ck("'s' efface une fois la cause disparue", safety_try_clear(&s,&in)==0u && !s.inhibit);

    safety_init(&s); in=nominal(); in.ibat_ma=-OC_TRIP_MA;
    ck("surintensite mesuree en recuperation (courant negatif) : verrouillee",
       run(&s,&in,1) && (s.latched & FLAG_OVERCURRENT));

    safety_init(&s); in=nominal(); in.ibat_ma=OC_SUSTAINED_MA+100;
    int n_ok=(int)(OC_SUSTAINED_MS/20u)-1;
    ck("seuil soutenu : pic court tolere", !run(&s,&in,n_ok));
    ck("seuil soutenu : verrouille apres OC_SUSTAINED_MS", run(&s,&in,1));
    safety_init(&s); in=nominal();
    in.ibat_ma=OC_SUSTAINED_MA+100; run(&s,&in,n_ok);
    in.ibat_ma=1000; run(&s,&in,1);
    in.ibat_ma=OC_SUSTAINED_MA+100;
    ck("seuil soutenu : la minuterie repart de zero apres un retour sous le seuil",
       !run(&s,&in,n_ok));
    in.ibat_ma=OC_SUSTAINED_MA+100; run(&s,&in,5);
    safety_try_clear(&s,&in);
    ck("'s' refuse tant que le courant reste au-dessus du seuil soutenu",
       s.latched & FLAG_OVERCURRENT);

    puts("\nSous-tension :");
    safety_init(&s); in=nominal(); in.vbat_mv=UV_TRIP_MV-500;
    int n_uv=(int)(UV_DEBOUNCE_MS/20u)-1;
    ck("creux de tension court (acceleration) : tolere", !run(&s,&in,n_uv));
    ck("verrouillee apres UV_DEBOUNCE_MS", run(&s,&in,1) && (s.latched & FLAG_UNDERVOLTAGE));
    in.vbat_mv=UV_TRIP_MV+500;
    safety_try_clear(&s,&in);
    ck("'s' refuse dans la zone d'hysteresis", s.latched & FLAG_UNDERVOLTAGE);
    in.vbat_mv=UV_CLEAR_MV;
    ck("'s' efface au-dessus de UV_CLEAR_MV", safety_try_clear(&s,&in)==0u);

    puts("\nBatterie debranchee (robot sur USB seul) :");
    safety_init(&s); in=nominal(); in.vbat_mv=40; in.ibat_ma=0;
    ck("pas de sous-tension verrouillee", !run(&s,&in,500) && s.latched==0u);
    ck("drapeau NO_BATTERY pose", safety_flags(&s,&in) & FLAG_NO_BATTERY);
    in.vbat_mv=19500;
    run(&s,&in,1);
    ck("batterie rebranchee : drapeau retombe sans acquittement",
       !(safety_flags(&s,&in) & FLAG_NO_BATTERY) && !s.inhibit);

    puts("\nINA226 absent :");
    safety_init(&s); in=nominal(); in.ina_present=false; in.vbat_mv=0; in.ibat_ma=0;
    bool inh=run(&s,&in,10);
    ck(INA_REQUIRED ? "INA_REQUIRED=1 : moteurs interdits" : "INA_REQUIRED=0 : le robot roule",
       inh==(bool)INA_REQUIRED);
    ck("drapeau INA_ABSENT pose", safety_flags(&s,&in) & FLAG_INA_ABSENT);
    ck("pas de faux NO_BATTERY sans capteur", !(safety_flags(&s,&in) & FLAG_NO_BATTERY));

    puts("\nDrapeau MOTORS_INHIBITED :");
    safety_init(&s); in=nominal(); in.alert_fired=true; run(&s,&in,1);
    ck("pose des qu'un defaut est verrouille", safety_flags(&s,&in) & FLAG_MOTORS_INHIBITED);

    printf("\n%d cas, %d echec(s)\n",total,fails);
    return fails!=0;
}
