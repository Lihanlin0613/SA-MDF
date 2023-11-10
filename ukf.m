p=csvread("chlog3 (1).csv",1,3);
p(:,10)=p(:,10);
p(:,11)=p(:,11);
p(:,12)=p(:,12);                                                 
p1=p(:,10:12);
p2=deg2rad(p1);
p3=eul2rotm(p2);
a2=p(:,1:3);
a2=a2(1:1700,:)
a3=array2table(a2);
Ta3=table2timetable(a3,'SampleRate',100);
ay=(a2(:,2));
ax=a2(:,1)；
v=cumtrapz(0.01,ay);
xx=cumtrapz(0.01,cumtrapz(0.01,ay));
figure
plot(0:0.01:16.99,xx)
<<<<<<< HEAD
w=-deg2rad(p(1:1700,4));
=======
w=-deg2rad(p(1:1700,4))；
>>>>>>> d626b19797e63cb7ab6d159bf70672bc8e005f24
for i=1:1699
    if(w(i)==0)
        w(i)=0.01;
    end
end
odom=xlsread("odom.xlsx");
odom1=((-odom(1:17,5))/10); %%dataset import
dt=0.01;
k=1;
N=1700;
x1=zeros(5,1700);
i=1;
x1(5,1)=w(1);
nx=5;
lamd=3-nx;
k=1;
for i=1:1:1699
    k=k+1;
    x1(3,k)=v(k);
    x1(4,k)=x1(4,k-1)+x1(5,k-1)*dt;
    x1(5,k)=w(k);
    x1(1,k)=x1(1,k-1)+(x1(3,k-1)/x1(5,k-1))*(sin(x1(5,k-1)*dt+x1(4,k-1))-sin(x1(4,k-1)));
    x1(2,k)=x1(2,k-1)+(x1(3,k-1)/x1(5,k-1))*(-cos(x1(5,k-1)*dt+x1(4,k-1))+cos(x1(4,k-1)));
end
P=eye(5);
p2=sqrt(P);
x2(:,1)=x1(:,1);
for i=0:1:1699
    for j=1:1:5
    x2(:,i*11+1)=x1(:,i+1);
    x2(:,i*11+1+j)=x1(:,i+1)+sqrt(lamd+5)*p2(:,j);
    x2(:,i*11+1+j+5)=x1(:,i+1)-sqrt(lamd+5)*p2(:,j);
    end
end %% UT Transformer
xp=zeros(5,1700);
xs=zeros(5,18700);
w0=0.8/(5+0.745);
w1=0.8/(2*(5+0.745));
Q = [0.00001 0 0 0 0; 0 0.00001 0 0 0;0 0 0.000001 0 0;0 0 0 0.00001 0;0 0 0 0 0.00001 ]; 
 
for i=1:1:18700
    if(mod(i-1,11)==0)
        xs(:,i)=w0*x2(:,i);
    else
        xs(:,i)=w1*x2(:,i);
    end
end
k=1;
for i=1:1:1700
    xss=0;
    for j=1:1:11
        xss=xss+xs(:,(k-1)*11+j);
    end
    xp(:,k)=xss;
    k=k+1;
end
k=1;
xr=zeros(5,1700);
H=[1 0 0 0 0];
P=eye(5);
P2=eye(5);
x3=zeros(5,11);
xl=zeros(5,11);
k=0;
xr(:,1)=0.01;
for i=1:1:1699
    k=k+1;
    xr(3,k+1)=xp(3,k);
    xr(4,k+1)=xr(4,k)+xr(5,k)*dt;
    xr(5,k+1)=xp(5,k);
    xr(1,k+1)=xr(1,k)+(xr(3,k)/xr(5,k))*(sin(xr(5,k)*dt+xr(4,k))-sin(xr(4,k)));
    xr(2,k+1)=xr(2,k)+(xr(3,k)/xr(5,k))*(-cos(xr(5,k)*dt+xr(4,k))+cos(xr(4,k)));
    if(mod(k,100))==0
    P =  P * P'+Q ;
    K = (P*H')*inv(H*P*H' + 0.001);
    xr(:,k) = xr(:,k) + K * (odom1((k+100)/100)-H*xr(:,k));
    P = (eye(5)-K*H) * P;
    end
<<<<<<< HEAD
end %% forward predection
for i=1:1:1698
    k=k-1;
    xr(3,k+1)=xr(3,k);
    xr(4,k+1)=xr(4,k)+xr(5,k)*dt;
    xr(5,k+1)=xr(5,k);
    xr(1,k+1)=xr(1,k)+(xr(3,k)/xr(5,k))*(sin(xr(5,k)*dt+xr(4,k))-sin(xr(4,k)));
    xr(2,k+1)=xr(2,k)+(xr(3,k)/xr(5,k))*(-cos(xr(5,k)*dt+xr(4,k))+cos(xr(4,k)));
    if(mod(k,100))==0
    P2 =  P2 * P2'+Q ;
    K2 = (P2*H')*inv(H*P2*H' + 0.001);
    xr(:,k+1) = xr(:,k) + K2 * (odom1((k-100)/100+1)-H*xr(:,k));
    P2 = (eye(5)-K2*H) * P2;
    end
end %% backward filter


=======
end
>>>>>>> d626b19797e63cb7ab6d159bf70672bc8e005f24
figure
plot(0:0.01:16.99,xx,'b-');hold on;
plot(0:0.01:16.99,xr(1,:),'g-');hold on;                   
plot(0:1:16, odom1,'r-');                           
axis([0 23 0 5]);
axis([0 23 0 5]);