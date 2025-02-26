
function y = norm_pdf(x, mu, sigma)
    y = (1 / (sigma*sqrt(2*pi))) * exp(-1/2*((x-mu)/sigma).^2);
end