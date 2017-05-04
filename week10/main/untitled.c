#include <stdio.h>

int main() {
	int n;
	int a[1000][1000], b[1000][1000];
	int x[1000];
	int y[1000];
	
	int i,j,k;
	int t,p;
	int m = 0;
	scanf("%d",&n);
	for(i=0;i<n;i++) {
		for(j=0;j<n;++j) 
			scanf("%d",*(a+i)+j);
	}	
	for(i=0;i<n;i++) 
		for(j=0;j<n;++j) 
			scanf("%d",*(b+j)+i);	
	for(i=0;i<n;i++) {
		if(*(x+i)==1) {
			for(j=0;j<n;j++) scanf("%d",&t);
			continue;
		}
		for(j=0;j<n;j++) {
			scanf("%d",&t);
			if(*(x+i)==1) break;
			if(*(y+j)==1) continue;
			p = 0;
			for(k=0;k<n;k++) {
				p+=*(*(a+i)+k)*(*(*(b+j)+k));
			}
			if(t!=p) {
				printf("%d %d\n",i+1,j+1);
				*(x+i)=1;
				*(y+j)=1;
				m++;
			}
			if(m==n) return 0;
		}
		for(k=j+1;k<n;k++) scanf("%d",&t);
	}
	return 0;
}	