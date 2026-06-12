function [ V, d, it_num, rot_num ] = jacobi_eigenvalue ( A, it_max )

%*****************************************************************************80
%
%% jacobi_eigenvalue() carries out the Jacobi eigenvalue iteration.
%
%  Discussion:
%
%    This function computes the eigenvalues and eigenvectors of a
%    real symmetric matrix, using Rutishauser's modfications of the classical
%    Jacobi rotation method with threshold pivoting. 
%
%  Licensing:
%
%    This code is distributed under the GNU LGPL license.
%
%  Modified:
%
%    17 September 2013
%
%  Author:
%
%    John Burkardt
%
%  Reference:
%
%    Gene Golub, Charles VanLoan,
%    Matrix Computations,
%    Third Edition,
%    Johns Hopkins, 1996,
%    ISBN: 0-8018-4513-X,
%    LC: QA188.G65.
%
%  Input:
%
%    real A(N,N), the matrix, which must be square, real, and symmetric.
%
%    integer IT_MAX, the maximum number of iterations.
%
%  Output:
%
%    real V(N,N), the matrix of eigenvectors.
%
%    real D(N), the eigenvalues, in descending order.
%
%    integer IT_NUM, the total number of iterations.
%
%    integer ROT_NUM, the total number of rotations applied.
%
  n = size ( A, 1 );

  V = eye ( n, n );
  d = diag ( A );
  bw(1:n,1) = d(1:n,1);
  zw(1:n,1) = 0.0;
  it_num = 0;
  rot_num = 0;

  while ( it_num < it_max )

    it_num = it_num + 1;
%
%  The convergence threshold is based on the size of the elements in
%  the strict upper triangle of the matrix.
%
    thresh = sqrt ( sum ( sum ( triu ( A, 1 ).^2 ) ) ) / ( 4.0 * n );

    if ( thresh == 0.0 )
      break;
    end

    for p = 1 : n
      for q = p + 1 : n

        gapq = 10.0 * abs ( A(p,q) );
        termp = gapq + abs ( d(p) );
        termq = gapq + abs ( d(q) );
%
%  Annihilate tiny offdiagonal elements.
%
        if ( 4 < it_num && termp == abs ( d(p) ) && termq == abs ( d(q) ) )

          A(p,q) = 0.0;
%
%  Otherwise, apply a rotation.
%
        elseif ( thresh <= abs ( A(p,q) ) )

          h = d(q) - d(p);
          term = abs ( h ) + gapq;

          if ( term == abs ( h ) ) 
            t = A(p,q) / h;
          else
            theta = 0.5 * h / A(p,q);
            t = 1.0 / ( abs ( theta ) + sqrt ( 1.0 + theta * theta ) );
            if ( theta < 0.0 ) 
              t = - t;
            end
          end

          c = 1.0 / sqrt ( 1.0 + t * t );
          s = t * c;
          tau = s / ( 1.0 + c );
          h = t * A(p,q);
%
%  Accumulate corrections to diagonal elements.
%
          zw(p) = zw(p) - h;                  
          zw(q) = zw(q) + h;
          d(p) = d(p) - h;
          d(q) = d(q) + h;

          A(p,q) = 0.0;
%
%  Rotate, using information from the upper triangle of A only.
%
          for j = 1 : p - 1
            g = A(j,p);
            h = A(j,q);
            A(j,p) = g - s * ( h + g * tau );
            A(j,q) = h + s * ( g - h * tau );
          end

          for j = p + 1 : q - 1
            g = A(p,j);
            h = A(j,q);
            A(p,j) = g - s * ( h + g * tau );
            A(j,q) = h + s * ( g - h * tau );
          end

          for j = q + 1 : n
            g = A(p,j);
            h = A(q,j);
            A(p,j) = g - s * ( h + g * tau );
            A(q,j) = h + s * ( g - h * tau );
          end
%
%  Accumulate information in the eigenvector matrix.
%
          for j = 1 : n
            g = V(j,p);
            h = V(j,q);
            V(j,p) = g - s * ( h + g * tau );
            V(j,q) = h + s * ( g - h * tau );
          end

          rot_num = rot_num + 1;

        end

      end
    end

    bw(1:n,1) = bw(1:n,1) + zw(1:n,1);
    d(1:n,1) = bw(1:n,1);
    zw(1:n,1) = 0.0;

  end
%
%  Ascending sort the eigenvalues and eigenvectors.
%
  for k = 1 : n - 1

    m = k;

    for l = k + 1 : n
      if ( d(l) < d(m) )
        m = l;
      end
    end

    if ( m ~= k )

      t    = d(m);
      d(m) = d(k);
      d(k) = t;

      w        = V(1:n,m);
      V(1:n,m) = V(1:n,k);
      V(1:n,k) = w;

    end

  end

  return
end

