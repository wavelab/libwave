% nb_measurements = 100;
% nth_order = 10;
%
% function A = matrix_A(nth_order, t)
%     A = [];
%
%     % row by row
%     for m = 1: length(t)
%         % first column
%         A(m, 1) = 1;
%
%         % rest of the columns
%         for p = 2: nth_order
%             a_p = A(m, p - 1) * t(m);
%             A(m, p) = a_p;
%         end
%     end
% endfunction
%
%
% % test function
% t = linspace(-1.0, 1.0, nb_measurements);
% b = 1.0 ./ (1.0 + 25.0 * power(t, 2));
%
% a = -1.0:0.01:1.0;
% c = 1.0 ./ (1.0 + 25.0 * power(a, 2));
%
% % plot graphs
% fig = figure;
% plot(a, c, "-");
% legend("test function");
% print -djpg -color "-S200,200" plot.jpg;
%
%
% % polynomial fit
% % A = matrix_A(nth_order, t);
% % x = pinv(A) * transpose(b);
% % y = A * x;
%
% % % plot graphs
% % fig = figure;
% % plot(t, b, "o", t, y, "-");
% % legend("measurements", "polynomial fit")
% % pause;
