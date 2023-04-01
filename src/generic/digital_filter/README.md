# Butterworth

* The step for implementation is based on article 
  * [Butterworth filter](https://www.dsprelated.com/showarticle/1119.php)

  * [Highpass filters](https://www.dsprelated.com/showarticle/1135.php)

* Some important equations and steps from the article  shown as below

## Bilinear transform

- Convert H(s) to H(z)

$$\begin{equation*}
s = 2f_s \frac{1 - z^{-1}}{1 + z^{-1}}
\end{equation*}$$

$$\begin{equation*}
z = \frac{1 + s / (2f_s)}{1 - s / (2f_s)}
\end{equation*}$$

## Filter synthesis

### Example steps for low pass filter
1. Find the poles of the analog prototype filter with $\Omega_{c}$ = 1 rad/s
2. Find the corresponding analog frequency $F_c$. based on desired $f_c$
3. Scale the s-plane poles by $2πF_c$.
4. Transform the poles from the s-plane to the z-plane.
5. Add N zeros at z = -1.
6. Convert poles and zeros to polynomials with coefficients $a_n$ and $b_n$

#### Steps
1.
$$\begin{equation*}
p_{ak} = -\sin(\theta) + j\cos(\theta)
\end{equation*}$$

where

$$\begin{equation*}
\theta = \frac{(2k-1)\pi}{2N}
\end{equation*}$$

2.
$$\begin{equation*}
F_c = \frac{f_s}{\pi} \tan(\frac{\pi f_c}{f_s})
\end{equation*}$$

4.
$$\begin{equation*}
p_k = \frac{1 + p_{ak} / (2f_s)}{1 - p_{ak} / (2f_s)}, k = 1: N
\end{equation*}$$

5. Add N zeros at z = -1

$$\begin{equation*}
H(z) = K \frac{(z + 1) ^ N} {(z - p_0)(z-p_1)...(z - p_{N - 1})}
\end{equation*}$$