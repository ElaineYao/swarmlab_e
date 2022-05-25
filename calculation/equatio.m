% syms a b c x;
% eqn = a*x^2 +b*x +c == 0;
% 
% S = solve(eqn)
% S = solve(eqn, a)

% % symbolic root 
% syms x;
% eqn = x^3 +44.1*(x^2)+(b-25)*x+44.1 == 0;
% S = solve(eqn,x)

% numerical root
b = 0.05;
p = [1 44.1 b-25 44.1*b];
r = roots(p)

% plot the function
x = linspace(0,1, 10000);
q = x.^3 +44.1.*(x.^2)+(b-25).*x+44.1*b;
plot(x,q);

% % inequality -> unable to find explicit solution
% syms x;
% b = 1;
% cond = x^3 +44.1*(x^2)+-24*x+44.1 < 0;
% solve(cond, x)

% syms a b
% solve(2*a^2 + 5*b^2 - 4*a + 2*b > 9, a)

% 