package com.example.taxi.ui.mypage.boarding_list

import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.databinding.ItemTaxiImgListBinding
import com.example.taxi.databinding.ItemTaxiListBinding
import com.gun0912.tedpermission.provider.TedPermissionProvider.context

class BoardingListAdapter: RecyclerView.Adapter<BoardingListAdapter.BoaredTaxiListViewHolder>() {
    var boardedTaxiList = mutableListOf<BoardedTaxi>()
    lateinit var onBoaredTaxiClickListener: (View, Int) -> Unit
    lateinit var context: Context

    fun setListData(data: MutableList<BoardedTaxi>){
        boardedTaxiList = data
    }

    fun updateList(list: MutableList<BoardedTaxi>){
        this.boardedTaxiList = list
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): BoaredTaxiListViewHolder {
        return BoaredTaxiListViewHolder(
            ItemTaxiListBinding.inflate(
                LayoutInflater.from(parent.context),
                parent,
                false
            )
        ).apply {
            bindOnItemClickListener(onBoaredTaxiClickListener)
        }
    }

    override fun onBindViewHolder(holder: BoaredTaxiListViewHolder, position: Int) {
        holder.bind(boardedTaxiList[position])
    }

    override fun getItemCount(): Int {
        return boardedTaxiList.size
    }

    class BoaredTaxiListViewHolder(private val binding: ItemTaxiListBinding) :
        RecyclerView.ViewHolder(binding.root) {

        fun bind(data: BoardedTaxi) {
            if(data.carImage != ""){
                Glide.with(context)
                    .load(data.carImage)
                    .into(binding.imageMyPageBoardTaxi)
            }else{
                Glide.with(context)
                    .load(R.drawable.img_car)
                    .into(binding.imageMyPageBoardTaxi)
            }
            binding.textMyPageBoardTaxiNumber2.text = data.carNumber
            binding.ratingMyPageBoardTaxiRideComfort.rating = data.rideComfortAverage.toFloat()
            binding.ratingMyPageBoardTaxiCleanliness.rating = data.cleanlinessAverage.toFloat()
        }

        fun bindOnItemClickListener(onItemClickListener: (View, Int) -> Unit ) {
            binding.root.setOnClickListener {
                onItemClickListener(it, bindingAdapterPosition)
            }
        }
    }

}