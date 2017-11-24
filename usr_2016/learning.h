#ifndef LEARNING_H
#define LEARNING_H

class LRaceLine;

typedef struct {
    int startDiv;
    int endDiv;
    int count;
    int side;
    int successCount;
    int failureCount;
    int damageCount;
} DivRecord;

class Learning {
    public:
        Learning(LRaceLine *rl);
        ~Learning() { if (divRecord) free(divRecord); }
        void getOvertakeProbability(int div, int side, double myspeed, double opponentspeed, double distance, double *chanceofovertake, double *chanceofloss, double *chanceofdamage);
        int startOvertake(int div, int side, double myspeed, double opponentspeed, double distance, int damage, int position);
        void endOvertake(int damage, int position);

    private:
        int startDiv;
        double speed;
        double oppSpeed;
        double distance;
        int pos;
        int damage;
        int DRCount;
        int DRAlloc;
        DivRecord *divRecord;
};
        
#endif
