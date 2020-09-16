figure;
[U S V]=svd(adjWeights)
for i=1:6
UU(:,:,i)=U(:,i)*V(:,i)';
end

for i=1:6
UU(:,:,i)=(UU(:,:,i))./(max(max(UU(:,:,i)))-min(min(UU(:,:,i))));
end

figure
for i=1:6
subplot(3,3,i)
imagesc(UU(:,:,i))
end

subplot(337)
imagesc(UU(:,:,1).*UU(:,:,4))
subplot(338)
imagesc(UU(:,:,2).*UU(:,:,5))
subplot(339)
imagesc(UU(:,:,3).*UU(:,:,5))

title('eigen 1')
title('eigen 2')
title('eigen 3')
title('eigen 4')
title('eigen 5')
title('eigen 6')
title('conjunction of 1&&4')
title('conjunction of 2&&5')
title('conjunction of 1&&4')
title('conjunction of 2&&5')



imagesc(U(:,2)*V(:,2)')
subplot(333)
imagesc(U(:,3)*V(:,3)')
subplot(334)
imagesc(U(:,4)*V(:,4)')
subplot(335)
imagesc(U(:,5)*V(:,5)')
subplot(336)
imagesc(U(:,6)*V(:,6)')
subplot(337)
imagesc((U(:,1)*V(:,1)').*(U(:,4)*V(:,4)'))