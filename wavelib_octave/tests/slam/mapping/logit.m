% Log odds graph

p = 0:0.01:1;

logitp = log(p) - log(1-p);

plot(p,logitp)
title('Log odds (logit)')
xlabel('p')
ylabel('logit(p)')