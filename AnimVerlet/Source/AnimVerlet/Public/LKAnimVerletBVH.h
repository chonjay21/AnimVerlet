#pragma once
#include <CoreMinimal.h>
#include "LKAnimVerletBvhType.h"

template <typename T = void*>
class LKAnimVerletBVH
{
public:
	using LKBvhID = int32;	///for external use
	static constexpr LKBvhID NullID = -1;

public:
	void Initialize(const FLKAnimVerletBvhSettings& InSettings, int32 NumHint) 
	{ 
		Settings = InSettings; 
		Nodes.Reserve(NumHint); 
	}

	void Destroy()
	{
		Root = NullNode;
		FreeListHead = NullNode;
		Nodes.Reset();
	}

	LKBvhID Insert(const FLKAnimVerletBound& InAABB, const T& InUserData)
	{
		const int32 NodeID = AllocateNode();
		Nodes[NodeID].Parent = NullNode;
		Nodes[NodeID].Left = NullNode;
		Nodes[NodeID].Right = NullNode;
		Nodes[NodeID].Height = 0;
		Nodes[NodeID].UserData = InUserData;
		Nodes[NodeID].Box = MakeFatAABB(InAABB);

		InsertLeaf(NodeID);
		return NodeID;
	}

	void Remove(LKBvhID InID)
	{
		if (!IsValidNode(InID))
			return;

		RemoveLeaf(InID);
		FreeNode(InID);
	}

	/// Refresh AABB: If it's inside the existing fat AABB, leave it as is, if it's outside, reinsert it.(InDisplacement: Movement of this frame)
	bool Update(LKBvhID InID, const FLKAnimVerletBound& InAABB, const FVector& InDisplacement)
	{
		if (IsValidNode(InID) == false)
			return false;

		const FLKAnimVerletBound FatAABB = MakeFatAABB(InAABB);

		/// Increase fatAABB in the sweep direction to prevent frequent reinsertion
		FLKAnimVerletBound SweptAABB = FatAABB;
		const FVector D = InDisplacement * Settings.DisplacementMultiplier;
		SweptAABB.Min += FVector(FMath::Min(0.0f, D.X), FMath::Min(0.0f, D.Y), FMath::Min(0.0f, D.Z));
		SweptAABB.Max += FVector(FMath::Max(0.0f, D.X), FMath::Max(0.0f, D.Y), FMath::Max(0.0f, D.Z));

		/// Maintain tree structure if existing box contains new AABB
		if (Nodes[InID].Box.IsInsideOrOn(SweptAABB))
			return false;

		RemoveLeaf(InID);
		Nodes[InID].Box = SweptAABB;
		InsertLeaf(InID);
		return true;
	}

	T* GetUserData(LKBvhID InID)
	{
		return IsValidNode(InID) ? &Nodes[InID].UserData : nullptr;
	}

	FLKAnimVerletBound GetFatAABB(LKBvhID InID) const
	{
		return IsValidNode(InID) ? Nodes[InID].Box : FLKAnimVerletBound();
	}

	template<typename FuncType>
	void QueryAABB(const FLKAnimVerletBound& InAABB, FuncType&& InCallback) const
	{
		if (Root == NullNode)
			return;

		TArray<LKBvhID, TInlineAllocator<64>> Stack;
		Stack.Add(Root);

		while (Stack.Num() > 0)
		{
			const LKBvhID NodeID = Stack.Pop(EAllowShrinking::No);
			const FLKBvhNode<T>& Node = Nodes[NodeID];

			if (Node.Box.IsIntersect(InAABB) == false)
				continue;

			if (Node.IsLeaf())
			{
				/// Stop if the result of Callback is false
				if (InCallback(NodeID, Node.UserData) == false)
					return;
			}
			else
			{
				Stack.Add(Node.Left);
				Stack.Add(Node.Right);
			}
		}
	}

	template<typename FuncType>
	void RayCast(const FLKAnimVerletBvhRay& InRay, FuncType&& InCallback) const
	{
		if (Root == NullNode)
			return;

		TArray<LKBvhID, TInlineAllocator<64>> Stack;
		Stack.Add(Root);

		while (Stack.Num() > 0)
		{
			const LKBvhID NodeID = Stack.Pop(EAllowShrinking::No);
			const FLKBvhNode<T>& Node = Nodes[NodeID];

			float HitT = 0.0f;
			if (LKAnimVerletUtil::RayCastAABB(OUT HitT, InRay, Node.Box) == false)
				continue;

			if (Node.IsLeaf())
			{
				if (InCallback(NodeID, Node.UserData, HitT) == false)
					return;
			}
			else
			{
				Stack.Add(Node.Left);
				Stack.Add(Node.Right);
			}
		}
	}

	int32 GetNaxHeight() const
	{
		return (Root == NullNode) ? 0 : Nodes[Root].Height;
	}

private:
	bool IsValidNode(int32 NodeID) const
	{
		return Nodes.IsValidIndex(NodeID) && Nodes[NodeID].Height >= 0;
	}

	FLKAnimVerletBound MakeFatAABB(const FLKAnimVerletBound& InAABB) const
	{
		FLKAnimVerletBound Out = InAABB;
		Out.Min -= Settings.FatExtension;
		Out.Max += Settings.FatExtension;
		return Out;
	}

	int32 AllocateNode()
	{
		if (FreeListHead != NullNode)
		{
			const int32 NodeId = FreeListHead;
			FreeListHead = Nodes[NodeId].Parent;
			Nodes[NodeId].Parent = NullNode;
			Nodes[NodeId].Left = NullNode;
			Nodes[NodeId].Right = NullNode;
			Nodes[NodeId].Height = 0;
			Nodes[NodeId].UserData = T{};
			Nodes[NodeId].Box.Reset();
			return NodeId;
		}

		const int32 NewId = Nodes.AddDefaulted();
		Nodes[NewId].Height = 0;
		return NewId;
	}

	void FreeNode(int32 NodeID)
	{
		Nodes[NodeID].Height = -1;
		Nodes[NodeID].Parent = FreeListHead;
		Nodes[NodeID].UserData = T{};
		FreeListHead = NodeID;
	}

	void InsertLeaf(int32 LeafID)
	{
		if (Root == NullNode)
		{
			Root = LeafID;
			Nodes[Root].Parent = NullNode;
			return;
		}

		/// 1) Search best sibling
		const FLKAnimVerletBound LeafBound = Nodes[LeafID].Box;
		int32 Index = Root;

		while (Nodes[Index].IsLeaf() == false)
		{
			const int32 LeftID = Nodes[Index].Left;
			const int32 RightID = Nodes[Index].Right;

			const float Area = Nodes[Index].Box.GetSurfaceArea();
			const FLKAnimVerletBound Combined = FLKAnimVerletBound::Combine(Nodes[Index].Box, LeafBound);
			const float CombinedArea = Combined.GetSurfaceArea();

			/// Additional cost when combined
			const float CostParent = 2.0f * CombinedArea;

			/// Cost of expanding the current node
			const float InheritanceCost = 2.0f * (CombinedArea - Area);

			float CostLeft = 0.0f;
			{
				const FLKAnimVerletBound CombinedLeft = FLKAnimVerletBound::Combine(Nodes[LeftID].Box, LeafBound);
				if (Nodes[LeftID].IsLeaf())
					CostLeft = CombinedLeft.GetSurfaceArea() + InheritanceCost;
				else
					CostLeft = (CombinedLeft.GetSurfaceArea() - Nodes[LeftID].Box.GetSurfaceArea()) + InheritanceCost;
			}

			float CostRight = 0.0f;
			{
				const FLKAnimVerletBound CombinedRight = FLKAnimVerletBound::Combine(Nodes[RightID].Box, LeafBound);
				if (Nodes[RightID].IsLeaf())
					CostRight = CombinedRight.GetSurfaceArea() + InheritanceCost;
				else
					CostRight = (CombinedRight.GetSurfaceArea() - Nodes[RightID].Box.GetSurfaceArea()) + InheritanceCost;
			}

			/// Stop if it's cheaper to append with the parents
			if (CostParent < CostLeft && CostParent < CostRight)
				break;

			Index = (CostLeft < CostRight) ? LeftID : RightID;
		}

		const int32 SiblingID = Index;

		/// 2) Create new parent
		const int32 OldParentID = Nodes[SiblingID].Parent;
		const int32 NewParentID = AllocateNode();

		Nodes[NewParentID].Parent = OldParentID;
		Nodes[NewParentID].Box = FLKAnimVerletBound::Combine(LeafBound, Nodes[SiblingID].Box);
		Nodes[NewParentID].Height = Nodes[SiblingID].Height + 1;

		Nodes[NewParentID].Left = SiblingID;
		Nodes[NewParentID].Right = LeafID;

		Nodes[SiblingID].Parent = NewParentID;
		Nodes[LeafID].Parent = NewParentID;

		if (OldParentID == NullNode)
		{
			Root = NewParentID;
		}
		else
		{
			if (Nodes[OldParentID].Left == SiblingID)
				Nodes[OldParentID].Left = NewParentID;
			else
				Nodes[OldParentID].Right = NewParentID;
		}

		/// 3) Refresh Height/Bound to upward and balancing
		int32 CurrentID = Nodes[LeafID].Parent;
		while (CurrentID != NullNode)
		{
			CurrentID = Balance(CurrentID);

			const int32 LeftId = Nodes[CurrentID].Left;
			const int32 RightId = Nodes[CurrentID].Right;

			Nodes[CurrentID].Height = 1 + FMath::Max(Nodes[LeftId].Height, Nodes[RightId].Height);
			Nodes[CurrentID].Box = FLKAnimVerletBound::Combine(Nodes[LeftId].Box, Nodes[RightId].Box);

			CurrentID = Nodes[CurrentID].Parent;
		}
	}

	void RemoveLeaf(int32 LeafID)
	{
		if (LeafID == Root)
		{
			Root = NullNode;
			return;
		}

		const int32 ParentID = Nodes[LeafID].Parent;
		const int32 GrandParentID = Nodes[ParentID].Parent;
		const int32 SiblingID = (Nodes[ParentID].Left == LeafID) ? Nodes[ParentID].Right : Nodes[ParentID].Left;
		if (GrandParentID != NullNode)
		{
			/// Remove parent, connect sibling to grandparent
			if (Nodes[GrandParentID].Left == ParentID)
				Nodes[GrandParentID].Left = SiblingID;
			else
				Nodes[GrandParentID].Right = SiblingID;

			Nodes[SiblingID].Parent = GrandParentID;
			FreeNode(ParentID);

			/// Refresh Height/Bound to upward and balancing
			int32 CurrentID = GrandParentID;
			while (CurrentID != NullNode)
			{
				CurrentID = Balance(CurrentID);

				const int32 LeftId = Nodes[CurrentID].Left;
				const int32 RightId = Nodes[CurrentID].Right;

				Nodes[CurrentID].Box = FLKAnimVerletBound::Combine(Nodes[LeftId].Box, Nodes[RightId].Box);
				Nodes[CurrentID].Height = 1 + FMath::Max(Nodes[LeftId].Height, Nodes[RightId].Height);

				CurrentID = Nodes[CurrentID].Parent;
			}
		}
		else
		{
			/// root == parent
			Root = SiblingID;
			Nodes[SiblingID].Parent = NullNode;
			FreeNode(ParentID);
		}

		Nodes[LeafID].Parent = NullNode;
	}

	/// Rotation like AVL tree
	int32 Balance(int32 InID)
	{
		FLKBvhNode<T>& A = Nodes[InID];
		if (A.IsLeaf() || A.Height < 2)
			return InID;

		const int32 BID = A.Left;
		const int32 CID = A.Right;

		FLKBvhNode<T>& B = Nodes[BID];
		FLKBvhNode<T>& C = Nodes[CID];

		const int32 BalanceFactor = C.Height - B.Height;

		/// Right heavy
		if (BalanceFactor > Settings.BalanceThreshold)
		{
			const int32 FID = C.Left;
			const int32 GID = C.Right;
			FLKBvhNode<T>& F = Nodes[FID];
			FLKBvhNode<T>& G = Nodes[GID];

			/// Rotate left: C becomes parent of A
			C.Left = InID;
			C.Parent = A.Parent;
			A.Parent = CID;

			if (C.Parent != NullNode)
			{
				if (Nodes[C.Parent].Left == InID) 
					Nodes[C.Parent].Left = CID;
				else 
					Nodes[C.Parent].Right = CID;
			}
			else
			{
				Root = CID;
			}

			/// Choose best subtree arrangement
			if (F.Height > G.Height)
			{
				C.Right = FID;
				A.Right = GID;
				G.Parent = InID;
				F.Parent = CID;

				A.Box = FLKAnimVerletBound::Combine(B.Box, G.Box);
				C.Box = FLKAnimVerletBound::Combine(A.Box, F.Box);

				A.Height = 1 + FMath::Max(B.Height, G.Height);
				C.Height = 1 + FMath::Max(A.Height, F.Height);
			}
			else
			{
				C.Right = GID;
				A.Right = FID;
				F.Parent = InID;
				G.Parent = CID;

				A.Box = FLKAnimVerletBound::Combine(B.Box, F.Box);
				C.Box = FLKAnimVerletBound::Combine(A.Box, G.Box);

				A.Height = 1 + FMath::Max(B.Height, F.Height);
				C.Height = 1 + FMath::Max(A.Height, G.Height);
			}

			return CID;
		}

		/// Left heavy
		if (BalanceFactor < -Settings.BalanceThreshold)
		{
			const int32 DID = B.Left;
			const int32 EID = B.Right;
			FLKBvhNode<T>& D = Nodes[DID];
			FLKBvhNode<T>& E = Nodes[EID];

			/// Rotate right: B becomes parent of A
			B.Right = InID;
			B.Parent = A.Parent;
			A.Parent = BID;

			if (B.Parent != NullNode)
			{
				if (Nodes[B.Parent].Left == InID) 
					Nodes[B.Parent].Left = BID;
				else 
					Nodes[B.Parent].Right = BID;
			}
			else
			{
				Root = BID;
			}

			/// Choose best subtree arrangement
			if (D.Height > E.Height)
			{
				B.Left = DID;
				A.Left = EID;
				E.Parent = InID;
				D.Parent = BID;

				A.Box = FLKAnimVerletBound::Combine(E.Box, C.Box);
				B.Box = FLKAnimVerletBound::Combine(D.Box, A.Box);

				A.Height = 1 + FMath::Max(E.Height, C.Height);
				B.Height = 1 + FMath::Max(D.Height, A.Height);
			}
			else
			{
				B.Left = EID;
				A.Left = DID;
				D.Parent = InID;
				E.Parent = BID;

				A.Box = FLKAnimVerletBound::Combine(D.Box, C.Box);
				B.Box = FLKAnimVerletBound::Combine(E.Box, A.Box);

				A.Height = 1 + FMath::Max(D.Height, C.Height);
				B.Height = 1 + FMath::Max(E.Height, A.Height);
			}
			return BID;
		}
		return InID;
	}

private:
	static constexpr int32 NullNode = -1;

	template <typename U>
	struct FLKBvhNode
	{
	public:
		FLKAnimVerletBound Box;
		U UserData{};

		int32 Parent = NullNode;
		int32 Left = NullNode;
		int32 Right = NullNode;

		int32 Height = -1; /// FreeNode when -1

	public:
		bool IsLeaf() const
		{
			return Left == NullNode;
		}
	};

private:
	int32 Root = NullNode;
	int32 FreeListHead = NullNode;
	TArray<FLKBvhNode<T>> Nodes;

	FLKAnimVerletBvhSettings Settings;
};