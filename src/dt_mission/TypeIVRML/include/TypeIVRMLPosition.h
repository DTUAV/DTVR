
































#ifndef VDr0j
#define VDr0j
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>
#include <RMLPositionFlags.h>
#include <RMLVelocityFlags.h>
#include <RMLVector.h>
#include <TypeIVRMLPolynomial.h>
#include <TypeIVRMLThreadControl.h>
#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLVelocity.h>
#include <TypeIVRMLMovingAverageFilter.h>
#ifdef bzQVt
#include <pthread.h>
#endif
using namespace qPN_6;
















































































class TypeIVRMLPosition{public:













































TypeIVRMLPosition(const unsigned int&ggmbr,const double&nsANK,const unsigned int
&m4tf3=(0x13d8+4219-0x2453),const double&cnY6I=qIT2N);











~TypeIVRMLPosition(void);















































































int Yq2Ls(const RMLPositionInputParameters&zqVEb,RMLPositionOutputParameters*
SDTKM,const RMLPositionFlags&Jdf48);


























































int Xvpsd(const double&PxS7M,RMLPositionOutputParameters*SDTKM);








int SetupOverrideFilter(const double&_IGXH,const double&kNQ3O);protected:





enum CYw8l{
omT9P=false,
T3CdJ=true};



















enum GP5LU{
kgwCF=(0x49a+759-0x791),



HtF6w=(0x19d7+2679-0x244d),


npWTr=(0xae4+619-0xd4d),


UR0Du=(0x18+5590-0x15eb),


eC32m=(0x1325+3139-0x1f64)};





























void ql7rg(void);
































void ePMhj(void);









































void e3pE4(RMLPositionOutputParameters*tg6GG)const;


























void hXYgE(void);















































bool RRkLb(void);











































bool kKWV_(void);








































int _XPNu(const double&PxS7M,const double&OverrideValue,
RMLPositionOutputParameters*tg6GG)const;






































































void Q48EP(const RMLPositionInputParameters&zqVEb,RMLPositionOutputParameters*
SDTKM,const RMLPositionFlags&XcMhX);




































bool ZSTS8(const double&HlaZx,const RMLDoubleVector&izcH4,const RMLDoubleVector&
yeziv)const;


































































bool tHCRc(RMLDoubleVector*fezGm);
#ifdef bzQVt










































static void*wfj20(void*);
#endif






























static void vN0kt(TypeIVRMLPosition*i75kT,unsigned int&bFhjT);






























static void fazSR(TypeIVRMLPosition*i75kT,unsigned int&bFhjT);































static void GahbM(TypeIVRMLPosition*i75kT,unsigned int&bFhjT);



























void ZeFtG(void);






















































void RQuk0(const double&PxS7M,const double&OverrideValue,
RMLPositionOutputParameters*tg6GG)const;














































bool q9K7w(const RMLPositionInputParameters&zqVEb,RMLPositionOutputParameters*
SDTKM);








































void chwJj(RMLPositionOutputParameters*tg6GG)const;





































void dwhIQ(void);
















void TTtbr(void);










bool F2ivR;









bool kfikd;









bool kj2Tc;







bool xheaR;







bool gHefb;











bool sg_1Y;








int ReturnValue;







unsigned int NumberOfDOFs;










unsigned int NumberOfOwnThreads;









unsigned int c5Mvm;








unsigned int zAQo1;










unsigned int EuH1E;









unsigned int Kv9sX;







double CycleTime;










double SynchronizationTime;









double zzKJk;













double Ghz_l;













double z2xUW;












double MaxTimeForOverrideFilter;







RMLPositionFlags XOquL;














RMLPositionFlags onYRt;












RMLBoolVector*GwQsU;











RMLBoolVector*wNCv9;













RMLBoolVector*Cc2Sm;










RMLVector<FOwyh>*qfrNo;












RMLVector<FOwyh>*bbZEf;
















RMLDoubleVector*LiWKZ;









RMLDoubleVector*Q_iq8;















RMLDoubleVector*Iwaq1;










RMLDoubleVector*F5Xo9;










RMLDoubleVector*ecwWM;









RMLDoubleVector*HxNxN;









RMLDoubleVector*Xmk8y;









RMLDoubleVector*adP3s;










RMLDoubleVector*dhSEu;









RMLDoubleVector*VmKWV;









RMLDoubleVector*BNvJP;









RMLDoubleVector*o04Be;










RMLDoubleVector*wdDbF;










RMLDoubleVector*TH6RH;










RMLDoubleVector*CrzAq;









RMLDoubleVector*jIU8s;









RMLDoubleVector*bqLBP;










RMLDoubleVector*v580I;











RMLDoubleVector*gE5PU;





RMLDoubleVector*NkYww;










BF1yT*EyjZT;















RMLPositionInputParameters*xObJM;









RMLPositionInputParameters*_DBry;













RMLPositionOutputParameters*gSTLu;














TypeIVRMLVelocity*RMLVelocityObject;
















RMLVelocityInputParameters*fCNQO;
















RMLVelocityOutputParameters*uqgIZ;
















RMLVelocityFlags YROav;











XkwFr*Polynomials;
#ifdef bzQVt









pthread_cond_t waCo8;









pthread_mutex_t rdm6I;












bool VR_Fr;











pthread_t*LXv3E;
#endif













GP5LU IVKuG;












Dt6QZ*E24br;};
#endif

